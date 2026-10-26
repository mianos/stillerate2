#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#include "JsonWrapper.h"
#include "WebServer.h"

// stillerate2 HTTP control surface, layered on the shared WebServer base
// (/reset, /set_hostname, /healthz). Adds:
//   POST /firmware       raw .bin body -> inactive OTA slot -> reboot
//   GET  /firmware       running image version / partition
class StillWebServer : public WebServer {
public:
    explicit StillWebServer(WebContext* ctx);

    esp_err_t start() override;

protected:
    void populate_healthz_fields(WebContext* ctx, JsonWrapper& json) override;

private:
    static esp_err_t firmware_post_handler(httpd_req_t* req);
    static esp_err_t firmware_get_handler(httpd_req_t* req);
};
