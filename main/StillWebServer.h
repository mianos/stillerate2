#pragma once

#include <mutex>
#include <string>

#include "esp_err.h"
#include "esp_http_server.h"

#include "JsonWrapper.h"
#include "Max31865Sensor.h"
#include "StillContext.h"
#include "WebServer.h"

// stillerate2 HTTP control surface, layered on the shared WebServer base
// (/reset, /set_hostname, /healthz). All commands are REST; MQTT carries
// telemetry only. Every POST logs the caller's address.
//   GET/POST /pid            PID parameters, loop state, last run change
//   POST     /pid/run        {"sample_time": s}, 0 stops the loop
//   POST     /pid/reset      clear integral and derivative history
//   GET/POST /emulation      {"enabled", "temp"}
//   GET/POST /settings       device settings (password masked)
//   GET      /sensors        raw, ohms, temperature and faults per RTD channel
//   POST     /sensors/reset  reset both MAX31865s, then report as GET
//   POST     /firmware       raw .bin body -> inactive OTA slot -> reboot
//   GET      /firmware       running image version / partition
class StillWebServer : public WebServer {
public:
    StillWebServer(WebContext* ctx, StillContext* still, Max31865Sensor* reflux, Max31865Sensor* boiler);

    esp_err_t start() override;

protected:
    void populate_healthz_fields(WebContext* ctx, JsonWrapper& json) override;

private:
    static esp_err_t firmware_post_handler(httpd_req_t* req);
    static esp_err_t firmware_get_handler(httpd_req_t* req);
    static esp_err_t sensors_get_handler(httpd_req_t* req);
    static esp_err_t sensors_reset_handler(httpd_req_t* req);
    static esp_err_t pid_get_handler(httpd_req_t* req);
    static esp_err_t pid_post_handler(httpd_req_t* req);
    static esp_err_t pid_run_handler(httpd_req_t* req);
    static esp_err_t pid_reset_handler(httpd_req_t* req);
    static esp_err_t emulation_get_handler(httpd_req_t* req);
    static esp_err_t emulation_post_handler(httpd_req_t* req);
    static esp_err_t settings_get_handler(httpd_req_t* req);
    static esp_err_t settings_post_handler(httpd_req_t* req);

    void addPidState(JsonWrapper& json);

    StillContext* still_;
    Max31865Sensor* reflux_;
    Max31865Sensor* boiler_;

    std::mutex lastRunMu_;
    double lastRunSampleTime_ = -1.0;  // -1: no change since boot
    std::string lastRunFrom_;
    std::string lastRunAt_;
};
