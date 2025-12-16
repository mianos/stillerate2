#pragma once

#include <string>
#include <optional>

#include "HttpClient.h"
#include "JsonWrapper.h"

class RESTMotorController {
private:
    HttpClient httpClient;
    std::string motorName;

public:
    explicit RESTMotorController(const std::string& url, std::optional<std::string> name = std::nullopt)
        : httpClient(url), motorName(name.value_or("")) {}

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
