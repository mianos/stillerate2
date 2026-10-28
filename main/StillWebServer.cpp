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

#include "Emulation.h"
#include "PIDController.h"
#include "PidControlTimer.h"
#include "SettingsManager.h"

namespace {

constexpr const char* TAG = "stillweb";

std::string now_iso() {
    char buf[32];
    time_t now = time(nullptr);
    struct tm tm;
    localtime_r(&now, &tm);
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", &tm);
    return buf;
}


}  // namespace

StillWebServer::StillWebServer(WebContext* ctx, StillContext* still, Max31865Sensor* middle, Max31865Sensor* head)
    : WebServer(ctx), still_(still), middle_(middle), head_(head) {}

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
    ESP_LOGW(TAG, "POST /firmware from %s", callerAddress(req).c_str());
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
    resp.AddItem("written",   written);
    resp.AddItem("partition", std::string(target->label));
    resp.AddItem("rebooting", true);
    sendJson(req, resp);

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
    add_sensor_fields(resp, "middle", self->middle_);
    add_sensor_fields(resp, "head", self->head_);
    return sendJson(req, resp);
}

esp_err_t StillWebServer::sensors_reset_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    ESP_LOGW(TAG, "POST /sensors/reset from %s", callerAddress(req).c_str());
    JsonWrapper resp;
    if (self->middle_) resp.AddItem("middle_reset", std::string(esp_err_to_name(self->middle_->reset())));
    if (self->head_) resp.AddItem("head_reset", std::string(esp_err_to_name(self->head_->reset())));
    add_sensor_fields(resp, "middle", self->middle_);
    add_sensor_fields(resp, "head", self->head_);
    return sendJson(req, resp);
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
    return sendJson(req, resp);
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
    return sendJson(req, resp);
}

// Invalid values are rejected field by field (see PIDController); the reply
// always carries the parameters actually in force.
esp_err_t StillWebServer::pid_post_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper json;
    if (!readJsonBody(req, json)) return ESP_FAIL;
    ESP_LOGW(TAG, "POST /pid from %s: %s", callerAddress(req).c_str(), json.ToString().c_str());
    if (!self->still_->pid->setParametersFromJsonWrapper(json)) {
        return sendJsonError(req, 400, "no valid PID parameters in request");
    }
    JsonWrapper resp;
    self->addPidState(resp);
    return sendJson(req, resp);
}

esp_err_t StillWebServer::pid_run_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper json;
    if (!readJsonBody(req, json)) return ESP_FAIL;
    const std::string from = callerAddress(req);
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
    return sendJson(req, resp);
}

esp_err_t StillWebServer::pid_reset_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    ESP_LOGW(TAG, "POST /pid/reset from %s", callerAddress(req).c_str());
    self->still_->pid->reset();
    JsonWrapper resp;
    self->addPidState(resp);
    resp.AddItem("reset", true);
    return sendJson(req, resp);
}

esp_err_t StillWebServer::emulation_get_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper resp;
    self->still_->emu->addTo(resp);
    return sendJson(req, resp);
}

esp_err_t StillWebServer::emulation_post_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper json;
    if (!readJsonBody(req, json)) return ESP_FAIL;
    ESP_LOGW(TAG, "POST /emulation from %s: %s", callerAddress(req).c_str(), json.ToString().c_str());
    if (!self->still_->emu->fromJsonWrapper(json)) {
        return sendJsonError(req, 400, "need 'enabled' (bool) and/or 'temp' (-50..300 C)");
    }
    JsonWrapper resp;
    self->still_->emu->addTo(resp);
    return sendJson(req, resp);
}

esp_err_t StillWebServer::settings_get_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper resp;
    self->still_->settings->toJsonWrapper(resp, /*maskPassword=*/true);
    return sendJson(req, resp);
}

// Network settings (broker, sensorName, pump URLs) take effect on reboot; the
// RTD calibration applies immediately.
esp_err_t StillWebServer::settings_post_handler(httpd_req_t* req) {
    auto* self = static_cast<StillWebServer*>(req->user_ctx);
    JsonWrapper json;
    if (!readJsonBody(req, json)) return ESP_FAIL;
    ESP_LOGW(TAG, "POST /settings from %s", callerAddress(req).c_str());
    const SettingsManager::ChangeList changes = self->still_->settings->updateFromJsonWrapper(json);
    self->still_->ptimer->applySensorCalibration();
    JsonWrapper resp;
    resp.AddItem("changed", static_cast<int>(changes.size()));
    for (const auto& [key, value] : changes) {
        resp.AddItem(key, key == "mqttPassword" ? std::string("***") : value);
    }
    return sendJson(req, resp);
}
