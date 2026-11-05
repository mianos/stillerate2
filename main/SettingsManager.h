#pragma once
#include <cmath>
#include <cstdlib>
#include <string>
#include <type_traits>
#include <vector>
#include <utility>

#include "esp_log.h"

#include "NvsStorageManager.h"
#include "JsonWrapper.h"

class SettingsManager {
    NvsStorageManager nvs;
public:
    using ChangeList = std::vector<std::pair<std::string, std::string>>;

    SettingsManager(NvsStorageManager& nvs) : nvs(nvs) {
        loadSettings();
    }

    std::string mqttBrokerUri = "mqtt://mqtt2.mianos.com";
    std::string mqttUserName = "";
    std::string mqttUserPassword = "";
    std::string sensorName = "still2";
    std::string tz = "AEST-10AEDT,M10.1.0,M4.1.0/3";
    std::string ntpServer = "time.google.com";
//    std::string refluxPump = "http://131.84.1.78:8080/pump";
//    std::string condenserPump = "http://131.84.1.78:8080/pump";
    std::string refluxPump = "http://reflux.mianos.com/pump";
    std::string condenserPump = "http://condenser.mianos.com/pump";
    std::string powerCutUrl = "http://iampwr.mianos.com/api/setpower?x=0";

    float dangerTemp = 98.5f;
    float dangerHysteresis = 3.0f;

    float middleRref   = 401.5f;
    float headRref     = 401.5f;
    float middleOffset = 0.0f;
    float headOffset   = 0.0f;

    // Which RTD the PID uses as its process variable. "head" (default) or
    // "middle" — lets you repoint the PID in software when the probes are moved.
    std::string pidSource = "head";

    std::string convertChangesToJson(const SettingsManager::ChangeList& changes) {
        cJSON *root = cJSON_CreateObject();
        for (const auto& [key, value] : changes) {
            cJSON_AddStringToObject(root, key.c_str(), value.c_str());
        }
        char *rawJson = cJSON_Print(root);
        std::string jsonResponse(rawJson);
        cJSON_Delete(root);
        free(rawJson); // cJSON_Print allocates memory that must be freed
        return jsonResponse;
    }

    void loadSettings() {
        std::string value;

        nvs.retrieve("mqttBrokerUri", mqttBrokerUri);
        nvs.retrieve("mqttUserName", mqttUserName);
        nvs.retrieve("mqttPassword", mqttUserPassword);
        nvs.retrieve("sensorName", sensorName);
        nvs.retrieve("tz", tz);
        nvs.retrieve("ntpServer", ntpServer);
        nvs.retrieve("refluxPump", refluxPump);
        nvs.retrieve("condenserPump", condenserPump);
        nvs.retrieve("powerCutUrl", powerCutUrl);
        retrieveFloat("dangerTemp", dangerTemp);
        retrieveFloat("dangerHyst", dangerHysteresis);
        retrieveFloat("middleRref", middleRref);
        retrieveFloat("headRref", headRref);
        retrieveFloat("middleOffset", middleOffset);
        retrieveFloat("headOffset", headOffset);
        nvs.retrieve("pidSource", pidSource);
    }

    void toJsonWrapper(JsonWrapper& json, bool maskPassword = false) const {
        json.AddItem("mqttBrokerUri", mqttBrokerUri);
        json.AddItem("mqttUserName", mqttUserName);
        json.AddItem("mqttPassword", maskPassword && !mqttUserPassword.empty() ? std::string("***") : mqttUserPassword);
        json.AddItem("sensorName", sensorName);
        json.AddItem("tz", tz);
        json.AddItem("ntpServer", ntpServer);
        json.AddItem("refluxPump", refluxPump);
        json.AddItem("condenserPump", condenserPump);
        json.AddItem("powerCutUrl", powerCutUrl);
        json.AddItem("dangerTemp", dangerTemp);
        json.AddItem("dangerHyst", dangerHysteresis);
        json.AddItem("middleRref", middleRref);
        json.AddItem("headRref", headRref);
        json.AddItem("middleOffset", middleOffset);
        json.AddItem("headOffset", headOffset);
        json.AddItem("pidSource", pidSource);
    }

    std::string toJson() const {
        JsonWrapper json;
        toJsonWrapper(json, /*maskPassword=*/true);
        return json.ToString();
    }

   ChangeList updateFromJsonString(const std::string& jsonString) {
        const JsonWrapper json = JsonWrapper::Parse(jsonString);
		return updateFromJsonWrapper(json);
   }

   ChangeList updateFromJsonWrapper(const JsonWrapper& json) {
        ChangeList changes;
        updateFieldIfChanged(json, "mqttBrokerUri", mqttBrokerUri, changes);
        updateFieldIfChanged(json, "mqttUserName", mqttUserName, changes);
        updateFieldIfChanged(json, "mqttPassword", mqttUserPassword, changes);
        updateFieldIfChanged(json, "sensorName", sensorName, changes);
        updateFieldIfChanged(json, "tz", tz, changes);
        updateFieldIfChanged(json, "ntpServer", ntpServer, changes);
        updateFieldIfChanged(json, "refluxPump", refluxPump, changes);
        updateFieldIfChanged(json, "condenserPump", condenserPump, changes);
        updateFieldIfChanged(json, "powerCutUrl", powerCutUrl, changes);
        updateFieldIfChanged(json, "dangerTemp", dangerTemp, changes);
        updateFieldIfChanged(json, "dangerHyst", dangerHysteresis, changes);
        updateFieldIfChanged(json, "middleRref", middleRref, changes);
        updateFieldIfChanged(json, "headRref", headRref, changes);
        updateFieldIfChanged(json, "middleOffset", middleOffset, changes);
        updateFieldIfChanged(json, "headOffset", headOffset, changes);
        updateFieldIfChanged(json, "pidSource", pidSource, changes);
        if (pidSource != "head" && pidSource != "middle") {
            ESP_LOGW("Settings", "pidSource '%s' invalid; resetting to 'head'", pidSource.c_str());
            pidSource = "head";
            nvs.store("pidSource", pidSource);
        }

        for (const auto& [key, value] : changes) {
            nvs.store(key, value);
        }
        return changes;
   }

private:
    void retrieveFloat(const char* key, float& field) {
        std::string value;
        if (!nvs.retrieve(key, value)) return;
        char* end = nullptr;
        const float parsed = std::strtof(value.c_str(), &end);
        if (end != value.c_str() && std::isfinite(parsed)) {
            field = parsed;
        } else {
            ESP_LOGE("Settings", "bad %s='%s'", key, value.c_str());
        }
    }

    template <typename T>
    void updateFieldIfChanged(const JsonWrapper& json, const std::string& key, T& field, SettingsManager::ChangeList& changes) {
        if (json.ContainsField(key)) {  // Only proceed if the key exists in the JSON
            T newValue;
            if (json.GetField(key, newValue)) {  // Successfully retrieved new value
                if (newValue != field) {
                    field = newValue;

                    // Log the change for response
                    if constexpr (std::is_same_v<T, std::string>) {
                        changes.emplace_back(key, field);
                    } else {
                        changes.emplace_back(key, std::to_string(field));
                    }
                }
            } else {
                ESP_LOGE("SettingsUpdate", "Failed to retrieve new value for %s", key.c_str());
            }
        }
    }

};

