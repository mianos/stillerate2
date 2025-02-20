#pragma once

#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <functional>
#include <string>

class OTAUpdater {
public:
    std::string url;
    std::function<void(int)> progress_callback;

    OTAUpdater(const std::string& ota_url)
        : url(ota_url),
          progress_callback([](int progress) {
              ESP_LOGI("OTA", "Progress: %d%%", progress);
          }) { }

    // Allow setting a custom progress callback
    void set_progress_callback(std::function<void(int)> callback) {
        progress_callback = std::move(callback);
    }

    void start(const std::string& optional_url = "") {
        pending_url = optional_url.empty() ? url : optional_url;

        xTaskCreate([](void* obj) {
            auto* updater = static_cast<OTAUpdater*>(obj);
            updater->perform_update(updater->pending_url);
            vTaskDelete(nullptr);
        }, "ota_update_task", 8192, this, 5, nullptr);
    }

    void perform_update(const std::string& ota_url) {
        esp_http_client_config_t http_config = {};
        http_config.url = ota_url.c_str();
        http_config.cert_pem = nullptr;
        http_config.skip_cert_common_name_check = true;

        esp_https_ota_config_t ota_config = {};
        ota_config.http_config = &http_config;

        ESP_LOGI("OTA", "Starting OTA from URL: %s", ota_url.c_str());

        esp_https_ota_handle_t ota_handle = nullptr;
        esp_err_t ret = esp_https_ota_begin(&ota_config, &ota_handle);

        if (ret != ESP_OK) {
            ESP_LOGE("OTA", "Failed to begin OTA: %s", esp_err_to_name(ret));
            return;
        }

        int last_reported_progress = -1;

        while (true) {
            ret = esp_https_ota_perform(ota_handle);

            if (ret == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
                int image_size = esp_https_ota_get_image_size(ota_handle);
                int received_size = esp_https_ota_get_image_len_read(ota_handle);

                if (image_size > 0) {
                    int progress_percent = (received_size * 100) / image_size;

                    // Report progress only if changed
                    if (progress_percent != last_reported_progress) {
                        progress_callback(progress_percent);
                        last_reported_progress = progress_percent;
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(50));  // Reduced delay for better throughput
            } else if (ret == ESP_OK) {
                ESP_LOGI("OTA", "Download complete");
                break;
            } else {
                ESP_LOGE("OTA", "OTA perform failed: %s", esp_err_to_name(ret));
                esp_https_ota_abort(ota_handle);
                return;
            }
        }

        if (esp_https_ota_finish(ota_handle) == ESP_OK) {
            ESP_LOGI("OTA", "OTA successful, restarting...");
            esp_restart();
        } else {
            ESP_LOGE("OTA", "OTA finish failed or image invalid");
        }
    }

private:
    std::string pending_url;
};

