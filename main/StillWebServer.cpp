#include "StillWebServer.h"

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cmath>
#include <string>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "Emulation.h"
#include "PIDController.h"
#include "PidControlTimer.h"
#include "SettingsManager.h"

namespace {

constexpr const char* TAG = "stillweb";
constexpr int kMaxJsonBody = 2048;

std::string caller(httpd_req_t* req) {
    struct sockaddr_in6 addr = {};
    socklen_t len = sizeof(addr);
    if (getpeername(httpd_req_to_sockfd(req), reinterpret_cast<struct sockaddr*>(&addr), &len) != 0) {
        return "?";
    }
    char buf[INET6_ADDRSTRLEN] = {};
    if (addr.sin6_family == AF_INET) {
        inet_ntop(AF_INET, &reinterpret_cast<struct sockaddr_in*>(&addr)->sin_addr, buf, sizeof(buf));
        return buf;
    }
    inet_ntop(AF_INET6, &addr.sin6_addr, buf, sizeof(buf));
    std::string s(buf);
    for (const char* prefix : {"::FFFF:", "::ffff:"}) {
        if (s.rfind(prefix, 0) == 0) return s.substr(7);
    }
    return s;
}

std::string now_iso() {
    char buf[32];
    time_t now = time(nullptr);
    struct tm tm;
    localtime_r(&now, &tm);
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", &tm);
    return buf;
}

esp_err_t send_json(httpd_req_t* req, const JsonWrapper& json) {
    httpd_resp_set_type(req, "application/json");
    std::string out = json.ToString();
    return httpd_resp_sendstr(req, out.c_str());
}

}  // namespace

StillWebServer::StillWebServer(WebContext* ctx, StillContext* still, Max31865Sensor* reflux, Max31865Sensor* boiler)
    : WebServer(ctx), still_(still), reflux_(reflux), boiler_(boiler) {}

esp_err_t StillWebServer::start() {
    esp_err_t r = WebServer::start();
    if (r != ESP_OK) return r;

    struct Route {
        const char*    uri;
        httpd_method_t method;
        esp_err_t (*handler)(httpd_req_t*);
    };
    // With the base server's three routes this is 15 of the 16 it allows.
    const std::array<Route, 12> routes = {{
        {"/firmware",      HTTP_POST, firmware_post_handler},
        {"/firmware",      HTTP_GET,  firmware_get_handler},
        {"/sensors",       HTTP_GET,  sensors_get_handler},
        {"/sensors/reset", HTTP_POST, sensors_reset_handler},
        {"/pid",           HTTP_GET,  pid_get_handler},
        {"/pid",           HTTP_POST, pid_post_handler},
        {"/pid/run",       HTTP_POST, pid_run_handler},
        {"/pid/reset",     HTTP_POST, pid_reset_handler},
        {"/emulation",     HTTP_GET,  emulation_get_handler},
        {"/emulation",     HTTP_POST, emulation_post_handler},
        {"/settings",      HTTP_GET,  settings_get_handler},
        {"/settings",      HTTP_POST, settings_post_handler},
    }};

    for (const Route& route : routes) {
        httpd_uri_t uri = {
            .uri      = route.uri,
            .method   = route.method,
            .handler  = route.handler,
            .user_ctx = this,
        };
        esp_err_t err = httpd_register_uri_handler(server, &uri);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "register %s %s: %s",
                route.method == HTTP_POST ? "POST" : "GET", route.uri, esp_err_to_name(err));
            return err;
        }
    }
    return ESP_OK;
}

void StillWebServer::populate_healthz_fields(WebContext*, JsonWrapper& json) {
    const esp_app_desc_t*  desc    = esp_app_get_description();
    const esp_partition_t* running = esp_ota_get_running_partition();
    json.AddItem("version",   std::string(desc->version));
    json.AddItem("partition", std::string(running->label));
    json.AddItem("heap_free", static_cast<int>(esp_get_free_heap_size()));
}

// POST /firmware — raw .bin body. Streams into the inactive OTA slot, sets it as
// the next boot partition, then reboots.
// Deploy: curl --data-binary @build/stillerate2.bin http://<host>/firmware
esp_err_t StillWebServer::firmware_post_handler(httpd_req_t* req) {
    if (req->content_len <= 0) return sendJsonError(req, 400, "Content-Length required");

    const esp_partition_t* target = esp_ota_get_next_update_partition(nullptr);
    if (target == nullptr) return sendJsonError(req, 500, "no OTA partition available");
    if (req->content_len > static_cast<int>(target->size)) {
        return sendJsonError(req, 413, "image larger than OTA partition");
    }
    ESP_LOGI(TAG, "OTA: writing %d bytes to %s @ 0x%" PRIx32,
        req->content_len, target->label, target->address);

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) return sendJsonError(req, 500, esp_err_to_name(err));

    char buf[1024];
    int remaining = req->content_len;
    int written = 0;
    while (remaining > 0) {
        int got = httpd_req_recv(req, buf, std::min<int>(remaining, static_cast<int>(sizeof(buf))));
        if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (got <= 0) {
            esp_ota_abort(handle);
            return sendJsonError(req, 400, "request body truncated");
        }
        err = esp_ota_write(handle, buf, got);
        if (err != ESP_OK) {
            esp_ota_abort(handle);
            return sendJsonError(req, 500, esp_err_to_name(err));
        }
        written   += got;
        remaining -= got;
    }

    err = esp_ota_end(handle);
    if (err != ESP_OK) return sendJsonError(req, 400, esp_err_to_name(err));
    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) return sendJsonError(req, 500, esp_err_to_name(err));

    ESP_LOGW(TAG, "OTA: %d bytes written to %s; rebooting", written, target->label);
    JsonWrapper resp;
    resp.AddItem("status",    std::string("ok"));
    resp.AddItem("written",   written);
    resp.AddItem("partition", std::string(target->label));
    send_json(req, resp);

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

namespace {

void add_sensor_fields(JsonWrapper& json, const std::string& prefix, Max31865Sensor* sensor) {
    if (sensor == nullptr) return;
    const Max31865Sensor::Diagnostics d = sensor->diagnostics();
    json.AddItem(prefix + "_err",    std::string(esp_err_to_name(d.err)));
    json.AddItem(prefix + "_rref",   d.rRef);
    json.AddItem(prefix + "_offset", d.offsetC);
    if (d.err != ESP_OK) return;
    json.AddItem(prefix + "_raw",    static_cast<int>(d.raw));
    json.AddItem(prefix + "_ohms",   d.ohms);
    json.AddItem(prefix + "_fault",  d.fault);
    json.AddItem(prefix + "_fault_status", static_cast<int>(d.faultStatus));
    if (std::isfinite(d.temperatureC)) json.AddItem(prefix + "_c", d.temperatureC);
}

}  // namespace

esp_err_t StillWebServer::sensors_get_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper resp;
    add_sensor_fields(resp, "reflux", self->reflux_);
    add_sensor_fields(resp, "boiler", self->boiler_);
    return send_json(req, resp);
}

esp_err_t StillWebServer::sensors_reset_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper resp;
    if (self->reflux_) resp.AddItem("reflux_reset", std::string(esp_err_to_name(self->reflux_->reset())));
    if (self->boiler_) resp.AddItem("boiler_reset", std::string(esp_err_to_name(self->boiler_->reset())));
    add_sensor_fields(resp, "reflux", self->reflux_);
    add_sensor_fields(resp, "boiler", self->boiler_);
    return send_json(req, resp);
}

esp_err_t StillWebServer::firmware_get_handler(httpd_req_t* req) {
    const esp_app_desc_t*  desc    = esp_app_get_description();
    const esp_partition_t* running = esp_ota_get_running_partition();
    JsonWrapper resp;
    resp.AddItem("version",   std::string(desc->version));
    resp.AddItem("idf_ver",   std::string(desc->idf_ver));
    resp.AddItem("date",      std::string(desc->date));
    resp.AddItem("time",      std::string(desc->time));
    resp.AddItem("partition", std::string(running->label));
    return send_json(req, resp);
}

bool StillWebServer::readJson(httpd_req_t* req, JsonWrapper& json) {
    if (req->content_len <= 0) {
        sendJsonError(req, 400, "JSON body required");
        return false;
    }
    if (req->content_len > kMaxJsonBody) {
        sendJsonError(req, 413, "request body too large");
        return false;
    }
    std::string body(req->content_len, '\0');
    int received = 0;
    while (received < req->content_len) {
        int got = httpd_req_recv(req, body.data() + received, req->content_len - received);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (got <= 0) {
            sendJsonError(req, 400, "request body truncated");
            return false;
        }
        received += got;
    }
    cJSON* parsed = cJSON_Parse(body.c_str());
    if (parsed == nullptr || !cJSON_IsObject(parsed)) {
        cJSON_Delete(parsed);
        sendJsonError(req, 400, "invalid JSON");
        return false;
    }
    cJSON_Delete(parsed);
    json = JsonWrapper::Parse(body);
    return true;
}

void StillWebServer::addPidState(JsonWrapper& json) {
    still_->pid->toJsonWrapper(json);
    const uint32_t ms = still_->ptimer->periodMs();
    json.AddItem("running", ms > 0);
    json.AddItem("sample_time", ms / 1000.0);
    still_->emu->addTo(json);
    std::lock_guard<std::mutex> lock(lastRunMu_);
    if (lastRunSampleTime_ >= 0.0) {
        json.AddItem("last_run_sample_time", lastRunSampleTime_);
        json.AddItem("last_run_from", lastRunFrom_);
        json.AddItem("last_run_at", lastRunAt_);
    }
}

esp_err_t StillWebServer::pid_get_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper resp;
    self->addPidState(resp);
    return send_json(req, resp);
}

// Invalid values are rejected field by field (see PIDController); the reply
// always carries the parameters actually in force.
esp_err_t StillWebServer::pid_post_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper json;
    if (!self->readJson(req, json)) return ESP_FAIL;
    ESP_LOGW(TAG, "POST /pid from %s: %s", caller(req).c_str(), json.ToString().c_str());
    if (!self->still_->pid->setParametersFromJsonWrapper(json)) {
        return sendJsonError(req, 400, "no valid PID parameters in request");
    }
    JsonWrapper resp;
    self->addPidState(resp);
    return send_json(req, resp);
}

esp_err_t StillWebServer::pid_run_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper json;
    if (!self->readJson(req, json)) return ESP_FAIL;
    const std::string from = caller(req);
    ESP_LOGW(TAG, "POST /pid/run from %s: %s", from.c_str(), json.ToString().c_str());

    double sample_time = -1.0;
    if (!json.GetField("sample_time", sample_time)) {
        return sendJsonError(req, 400, "missing 'sample_time' (seconds, 0 stops)");
    }
    if (!(sample_time == 0.0 || (sample_time >= 0.5 && sample_time <= 3600.0))) {
        return sendJsonError(req, 400, "'sample_time' must be 0 (stop) or 0.5..3600 s");
    }
    self->still_->ptimer->start(static_cast<uint32_t>(sample_time * 1000.0 + 0.5));
    {
        std::lock_guard<std::mutex> lock(self->lastRunMu_);
        self->lastRunSampleTime_ = sample_time;
        self->lastRunFrom_ = from;
        self->lastRunAt_ = now_iso();
    }
    JsonWrapper resp;
    self->addPidState(resp);
    return send_json(req, resp);
}

esp_err_t StillWebServer::pid_reset_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    ESP_LOGW(TAG, "POST /pid/reset from %s", caller(req).c_str());
    self->still_->pid->reset();
    JsonWrapper resp;
    self->addPidState(resp);
    resp.AddItem("reset", true);
    return send_json(req, resp);
}

esp_err_t StillWebServer::emulation_get_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper resp;
    self->still_->emu->addTo(resp);
    return send_json(req, resp);
}

esp_err_t StillWebServer::emulation_post_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper json;
    if (!self->readJson(req, json)) return ESP_FAIL;
    ESP_LOGW(TAG, "POST /emulation from %s: %s", caller(req).c_str(), json.ToString().c_str());
    if (!self->still_->emu->fromJsonWrapper(json)) {
        return sendJsonError(req, 400, "need 'enabled' (bool) and/or 'temp' (-50..300 C)");
    }
    JsonWrapper resp;
    self->still_->emu->addTo(resp);
    return send_json(req, resp);
}

esp_err_t StillWebServer::settings_get_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper resp;
    self->still_->settings->toJsonWrapper(resp, /*maskPassword=*/true);
    return send_json(req, resp);
}

// Network settings (broker, sensorName, pump URLs) take effect on reboot; the
// RTD calibration applies immediately.
esp_err_t StillWebServer::settings_post_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper json;
    if (!self->readJson(req, json)) return ESP_FAIL;
    ESP_LOGW(TAG, "POST /settings from %s", caller(req).c_str());
    const SettingsManager::ChangeList changes = self->still_->settings->updateFromJsonWrapper(json);
    self->still_->ptimer->applySensorCalibration();
    JsonWrapper resp;
    resp.AddItem("changed", static_cast<int>(changes.size()));
    for (const auto& [key, value] : changes) {
        resp.AddItem(key, key == "mqttPassword" ? std::string("***") : value);
    }
    return send_json(req, resp);
}
