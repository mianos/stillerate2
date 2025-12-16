#pragma once

#include <string>
#include <optional>
#include <algorithm> // Required for string replacement

#include "HttpClient.h"
#include "JsonWrapper.h"
#include "esp_log.h"

class RESTMotorController {
private:
    HttpClient httpClient;
    std::string motorName;
    std::string healthUrl;

    // Helper: Converts "http://ip:port/pump" to "http://ip:port/healthz"
    std::string generateHealthUrl(std::string url) {
        std::string target = "/pump";
        std::string replacement = "/healthz";
        size_t start_pos = url.find(target);
        if(start_pos != std::string::npos) {
            url.replace(start_pos, target.length(), replacement);
        } else {
            // Fallback: if URL doesn't end in /pump, just append /healthz
            if (!url.empty() && url.back() == '/') {
                url.pop_back();
            }
            url += "/healthz";
        }
        return url;
    }

public:
    explicit RESTMotorController(const std::string& url, std::optional<std::string> name = std::nullopt)
        : httpClient(url), motorName(name.value_or("")), healthUrl(generateHealthUrl(url)) {}

    /**
     * @brief checks connectivity to the pump via the /healthz endpoint
     */
    bool isHealthy() {
        // Create a temporary client for the health URL
        HttpClient healthClient(healthUrl);
        
        // Assumes HttpClient has a .get() method returning {bool success, string response}
        auto [success, response] = healthClient.get();
        
        if (!success) {
            ESP_LOGW("RESTMotor", "Health check failed for %s at %s", motorName.c_str(), healthUrl.c_str());
        }
        return success;
    }

    bool setDutyPercentage(float percentage) {
        if (percentage < 0.0f || percentage > 100.0f) {
            return false;
        }

        JsonWrapper payload;
        payload.AddItem("duty", percentage);

        if (!motorName.empty()) {
            payload.AddItem("name", motorName);
        }

        auto [success, response] = httpClient.post(payload.ToString());
        return success;
    }
};
