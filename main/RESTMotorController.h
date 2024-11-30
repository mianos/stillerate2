#pragma once

#include <string>
#include "HttpClient.h"

class RESTMotorController {
private:
    HttpClient httpClient;

public:
    explicit RESTMotorController(const std::string& url) : httpClient(url) {}

    bool setDutyPercentage(float percentage) {
        if (percentage < 0.0f || percentage > 100.0f) {
            return false;
        }

        std::string payload = "{ \"duty\": " + std::to_string(percentage) + " }";
        auto [success, response] = httpClient.post(payload);

        return success;
    }
};
