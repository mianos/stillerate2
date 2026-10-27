#pragma once
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "MqttClient.h"
#include "SettingsManager.h"
#include "Max31865Sensor.h"
#include "JsonWrapper.h"

// Publishes every probe to tele/<name>/temp as {"<probe>": C, "fault": bool},
// independent of whether the PID loop is running.
class SensorLoopTask {
public:
    using Probe = std::pair<const char*, Max31865Sensor*>;

    SensorLoopTask(MqttClient& mqtt_client, SettingsManager& settings,
                   std::vector<Probe> probes, int period_ms = 10000)
        : mqtt_client(mqtt_client), settings(settings),
          probes(std::move(probes)), period(pdMS_TO_TICKS(period_ms)) {
        // A task, not a timer callback: each conversion blocks ~70 ms, which
        // would stall the FreeRTOS timer service.
        if (xTaskCreate(taskEntry, "sensors", 4096, this, 5, &handle) != pdPASS) {
            ESP_LOGE("SensorLoopTask", "Failed to create task");
        }
    }

    ~SensorLoopTask() {
        if (handle) vTaskDelete(handle);
    }

private:
    MqttClient& mqtt_client;
    SettingsManager& settings;
    std::vector<Probe> probes;
    TickType_t period;
    TaskHandle_t handle = nullptr;

    static void taskEntry(void* arg) {
        auto* self = static_cast<SensorLoopTask*>(arg);
        TickType_t last = xTaskGetTickCount();
        for (;;) {
            self->publishAll();
            vTaskDelayUntil(&last, self->period);
        }
    }

    void publishAll() {
        const std::string topic = "tele/" + settings.sensorName + "/temp";
        for (const auto& [name, sensor] : probes) {
            const float c = sensor->measure();
            JsonWrapper json;
            json.AddItem(name, c);
            json.AddItem("fault", !std::isfinite(c));
            json.AddTime();
            mqtt_client.publish(topic, json.ToString());
        }
    }
};
