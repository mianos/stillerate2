#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#include "JsonWrapper.h"
#include "Max31865Sensor.h"
#include "WebServer.h"

// stillerate2 HTTP control surface, layered on the shared WebServer base
// (/reset, /set_hostname, /healthz). Adds:
//   POST /firmware       raw .bin body -> inactive OTA slot -> reboot
//   GET  /firmware       running image version / partition
//   GET  /sensors        raw, ohms, temperature and faults per RTD channel
//   POST /sensors/reset  reset both MAX31865s, then report as GET
class StillWebServer : public WebServer {
public:
    StillWebServer(WebContext* ctx, Max31865Sensor* reflux, Max31865Sensor* boiler);

    esp_err_t start() override;

protected:
    void populate_healthz_fields(WebContext* ctx, JsonWrapper& json) override;

private:
    static esp_err_t firmware_post_handler(httpd_req_t* req);
    static esp_err_t firmware_get_handler(httpd_req_t* req);
    static esp_err_t sensors_get_handler(httpd_req_t* req);
    static esp_err_t sensors_reset_handler(httpd_req_t* req);

    Max31865Sensor* reflux_;
    Max31865Sensor* boiler_;
};
