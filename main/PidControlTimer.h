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

class PIDControlTimer {
private:
    TimerHandle_t pidTimer = nullptr;
	PIDController& pid;
	MqttClient& mqtt_client;
	SettingsManager& settings;
	Max31865Sensor& reflux_temp_sensor;
	MotorController& reflux_cooling_motor;

    static void pidTimerCallback(TimerHandle_t xTimer) {
        auto *instance = static_cast<PIDControlTimer*>(pvTimerGetTimerID(xTimer));
        (void)instance->runPIDControl();
    }

    esp_err_t runPIDControl() {
		float reflux_temp = reflux_temp_sensor.measure();
		JsonWrapper tjs;

		tjs.AddItem("reflux", reflux_temp);
		tjs.AddTime();
		mqtt_client.publish(std::string("tele/") + settings.sensorName + "/temp", tjs.ToString());

		auto error = pid.set_point - reflux_temp;

		float output;
	    auto rjs = pid.compute(error, output);
		reflux_cooling_motor.setDutyPercentage(output);

		rjs.AddTime();
		mqtt_client.publish(std::string("tele/") + settings.sensorName + "/pid", rjs.ToString());
		return ESP_OK;
    }

public:
    PIDControlTimer(PIDController& pid,
			MqttClient& mqtt_client,
			SettingsManager &settings,
			Max31865Sensor& reflux_temp,
			MotorController& reflux_cooling_motor)
		: pid(pid), mqtt_client(mqtt_client),
		  settings(settings),
		  reflux_temp_sensor(reflux_temp),
		  reflux_cooling_motor(reflux_cooling_motor) {}

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
};


