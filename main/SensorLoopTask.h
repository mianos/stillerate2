#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"

#include "MqttClient.h"
#include "SettingsManager.h"
#include "Max31865Sensor.h"
#include "JsonWrapper.h"

class SensorLoopTask {
private:
    TimerHandle_t sensorTimer = nullptr;
    MqttClient& mqtt_client;
    SettingsManager& settings;
    Max31865Sensor& boiler_temp;

    static void timerCallback(TimerHandle_t xTimer) {
        // Retrieve the instance pointer from the timer ID
        auto *instance = static_cast<SensorLoopTask*>(pvTimerGetTimerID(xTimer));
        instance->publishSensors();
    }

    void publishSensors() {
        float boiler = boiler_temp.measure();
        //ESP_LOGI("SensorLoopTask", "Temperature: %.2f", boiler);
        JsonWrapper json;
		json.AddItem("boiler", boiler);
		json.AddTime();
        std::string topic = "tele/" + settings.sensorName + "/temp";
		mqtt_client.publish(topic, json.ToString());
    }

public:
    SensorLoopTask(MqttClient& mqtt_client, SettingsManager &settings, Max31865Sensor& boiler_temp, int period=30000)
        : mqtt_client(mqtt_client), settings(settings), boiler_temp(boiler_temp) {
        sensorTimer = xTimerCreate("SensorTimer", pdMS_TO_TICKS(period), pdTRUE, this, timerCallback);
        if (sensorTimer == nullptr) {
            ESP_LOGE("SensorLoopTask", "Failed to create timer");
        } else {
            if (xTimerStart(sensorTimer, 0) != pdPASS) {
                ESP_LOGE("SensorLoopTask", "Failed to start timer");
            }
        }
    }

    ~SensorLoopTask() {
        if (sensorTimer != nullptr) {
            xTimerStop(sensorTimer, portMAX_DELAY);
            xTimerDelete(sensorTimer, portMAX_DELAY);
        }
    }
};
