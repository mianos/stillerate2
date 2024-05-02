#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"

#include "MqttContext.h"
#include "Emulation.h"

class PIDControlTimer {
private:
    TimerHandle_t pidTimer = nullptr;
    class MqttContext *ctx = nullptr;  // Use a pointer to manage the context

    static void pidTimerCallback(TimerHandle_t xTimer) {
        auto *instance = static_cast<PIDControlTimer*>(pvTimerGetTimerID(xTimer));
        instance->runPIDControl();
    }

    void runPIDControl() {
        ESP_LOGI("PIDControlTimer", "Timer callback executing");
        if (ctx) {
            // Utilize context here as needed
            ; // ESP_LOGI("PID", "Context PID: %d", *(ctx->pid));  // Example usage
        }
    }

public:
    PIDControlTimer() {}

    ~PIDControlTimer() {
        if (pidTimer != nullptr) {
            xTimerDelete(pidTimer, portMAX_DELAY);
        }
    }

    void setContext(MqttContext *context) {
        ctx = context;
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


