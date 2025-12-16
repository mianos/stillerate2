#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"

#include "MqttClient.h"
#include "MqttContext.h"
#include "Emulation.h"
#include "SettingsManager.h"
#include "PidController.h"
#include "Max31865Sensor.h"
#include "RESTMotorController.h"
#include "esp_timer.h"  // Include esp_timer for time functions

class PIDControlTimer {
private:
    TimerHandle_t pidTimer = nullptr;
    PIDController& pid;
    MqttClient& mqtt_client;
    SettingsManager& settings;
    Max31865Sensor& reflux_temp_sensor;
    RESTMotorController& reflux_cooling_motor;
    Max31865Sensor& boiler_temp_sensor;
    Emulation& emu;

    // New members for time measurement:
    uint64_t lastTime = 0;         // in microseconds
    uint32_t periodMsStored = 0;   // stores the timer period in ms

    static void pidTimerCallback(TimerHandle_t xTimer) {
        auto *instance = static_cast<PIDControlTimer*>(pvTimerGetTimerID(xTimer));
        (void)instance->runPIDControl();
    }

    esp_err_t runPIDControl() {
        JsonWrapper tjs;
        float reflux_temp;

        if (emu.enabled) {
            reflux_temp = emu.temp;
            tjs.AddItem("emulation", true);
        } else {
            reflux_temp = reflux_temp_sensor.measure();
        }

        tjs.AddItem("reflux", reflux_temp);
        tjs.AddTime();
        mqtt_client.publish(std::string("tele/") + settings.sensorName + "/temp", tjs.ToString());

        // Calculate dt in seconds using esp_timer_get_time().
        uint64_t current_time = esp_timer_get_time(); // in microseconds
        double dt = 0.0;
        if (lastTime == 0) {
            // On first call, use the timer period as a default.
            dt = periodMsStored / 1000.0;
        } else {
            dt = (current_time - lastTime) / 1e6;
        }
        lastTime = current_time;

        double output;
        // Pass dt to the compute function.
        auto rjs = pid.compute(reflux_temp, dt, output);
        reflux_cooling_motor.setDutyPercentage(output);

        if (emu.enabled) {
            rjs.AddItem("emulation", true);
        }
        rjs.AddTime();
        mqtt_client.publish(std::string("tele/") + settings.sensorName + "/pid", rjs.ToString());
        return ESP_OK;
    }

public:
    PIDControlTimer(PIDController& pid,
                      MqttClient& mqtt_client,
                      SettingsManager &settings,
                      Max31865Sensor& reflux_temp,
                      RESTMotorController& reflux_cooling_motor,
                      Max31865Sensor& boiler_temp,
                      Emulation& emu)
        : pid(pid), mqtt_client(mqtt_client),
          settings(settings),
          reflux_temp_sensor(reflux_temp),
          reflux_cooling_motor(reflux_cooling_motor),
          boiler_temp_sensor(boiler_temp),
          emu(emu) {}

    ~PIDControlTimer() {
        if (pidTimer != nullptr) {
            xTimerDelete(pidTimer, portMAX_DELAY);
        }
    }

    void start(uint32_t periodMs) {
        if (pidTimer != nullptr) {
            ESP_LOGI("PIDControlTimer", "Stopping and deleting old timer");
            xTimerStop(pidTimer, portMAX_DELAY);
            xTimerDelete(pidTimer, portMAX_DELAY);
            pidTimer = nullptr;
        }
        periodMsStored = periodMs;
        lastTime = 0; // Reset time on start.
        if (periodMs == 0) {
            ESP_LOGI("PIDControlTimer", "Timer stopped and not restarted (periodMs is 0)");
            return;
        }
        pidTimer = xTimerCreate("PIDTimer", pdMS_TO_TICKS(periodMs), pdTRUE, this, pidTimerCallback);
        if (pidTimer == nullptr) {
            ESP_LOGE("PIDControlTimer", "Failed to create timer");
        } else {
            if (xTimerStart(pidTimer, 0) != pdPASS) {
                ESP_LOGE("PIDControlTimer", "Failed to start timer");
            } else {
                ESP_LOGI("PIDControlTimer", "Timer started with a period of %u ms", (unsigned int)periodMs);
            }
        }
    }
    
    void resetAll() {
        reflux_temp_sensor.reset();
        boiler_temp_sensor.reset();
    }
};

