#pragma once

#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class OTAUpdater {
public:
    static constexpr const char* TAG = "OTA";

    void start() {
        xTaskCreate([](void* obj) {
            static_cast<OTAUpdater*>(obj)->perform_update();
            vTaskDelete(nullptr);
        }, "ota_update_task", 8192, this, 5, nullptr);
    }

private:
    void perform_update() {
        esp_http_client_config_t http_config = {};
        http_config.url = "http://your-server-address/firmware.bin"; // Replace with your firmware URL
        http_config.cert_pem = nullptr; // Set to server certificate if using HTTPS

        esp_https_ota_config_t ota_config = {};
        ota_config.http_config = &http_config; // Correctly pass the HTTP client config

        ESP_LOGI(TAG, "Starting OTA...");

        esp_err_t ret = esp_https_ota(&ota_config);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "OTA successful, restarting...");
            esp_restart();
        } else {
            ESP_LOGE(TAG, "OTA failed");
        }
    }
};
