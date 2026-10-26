#pragma once
#include <atomic>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "MqttClient.h"
#include "MqttContext.h"
#include "Emulation.h"
#include "SettingsManager.h"
#include "PidController.h"
#include "Max31865Sensor.h"
#include "RESTMotorController.h"
#include "esp_timer.h"  // Include esp_timer for time functions

// Runs the reflux PID loop on its OWN FreeRTOS task (not a software timer).
// runPIDControl() does blocking I/O — a sensor SPI read, MQTT publishes, and an
// HTTP POST to the pump (up to the HttpClient timeout). On a software timer that
// would block the shared timer-service daemon (stalling every other timer,
// including the sensor-publish loop). On a dedicated task, a slow/blocked pump
// POST only delays this loop's next iteration; nothing else is affected.
//
// Keeps the historical name and public API (start/resetAll). start(periodMs)
// (re)starts the loop at that period; start(0) stops it. Period changes and
// stop/start take effect promptly via a task notification.
class PIDControlTimer {
private:
    TaskHandle_t pidTaskHandle_ = nullptr;
    std::atomic<uint32_t> periodMs_{0};   // loop period in ms; 0 = stopped
    std::atomic<bool>     running_{false};

    PIDController& pid;
    MqttClient& mqtt_client;
    SettingsManager& settings;
    Max31865Sensor& reflux_temp_sensor;
    RESTMotorController& reflux_cooling_motor;
    Max31865Sensor& boiler_temp_sensor;
    Emulation& emu;

    // Touched only by the PID task, so no synchronisation needed.
    uint64_t lastTime = 0;         // in microseconds
    double   last_output_ = 0.0;
    int      consecutive_faults_ = 0;

    // Plausible reflux-temperature window (degrees C). Anything outside this, or
    // non-finite (the sensor returns NaN on a read failure), is treated as a
    // fault. Reflux temps live in roughly 0..100 C; the margin covers transients.
    static constexpr float kMinValidTemp = -40.0f;
    static constexpr float kMaxValidTemp = 200.0f;

    static void pidTaskTrampoline(void* arg) {
        static_cast<PIDControlTimer*>(arg)->taskLoop();
    }

    // Dedicated-task scheduler. Waits one period between runs; a notification
    // (from start()) interrupts the wait so a stop or period change is picked up
    // immediately. While stopped, blocks indefinitely until started.
    void taskLoop() {
        for (;;) {
            if (!running_.load()) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // sleep until start()
                continue;
            }
            const uint32_t p = periodMs_.load();
            // Interruptible wait of one period; nonzero return means we were
            // notified (state changed) — re-evaluate without running this cycle.
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(p ? p : 1000)) != 0) {
                continue;
            }
            if (!running_.load()) {
                continue;
            }
            (void)runPIDControl();
        }
    }

    esp_err_t runPIDControl() {
        JsonWrapper tjs;
        float reflux_temp;
        bool valid = true;

        if (emu.enabled) {
            reflux_temp = emu.temp;
            tjs.AddItem("emulation", true);
        } else {
            reflux_temp = reflux_temp_sensor.measure();
            valid = std::isfinite(reflux_temp) &&
                    reflux_temp > kMinValidTemp && reflux_temp < kMaxValidTemp;
        }

        tjs.AddItem("reflux", reflux_temp);
        tjs.AddItem("fault", !valid);
        tjs.AddTime();
        mqtt_client.publish(std::string("tele/") + settings.sensorName + "/temp", tjs.ToString());

        // On a sensor fault, do NOT run the PID (a garbage reading would corrupt
        // the integral and spike the derivative). Hold the last good output so the
        // pump keeps running, and report the fault.
        if (!valid) {
            ++consecutive_faults_;
            ESP_LOGW("PIDControlTimer",
                     "Invalid reflux temperature (%.2f); holding output at %.1f%% (fault #%d)",
                     reflux_temp, last_output_, consecutive_faults_);
            reflux_cooling_motor.setDutyPercentage(last_output_);
            JsonWrapper fjs;
            fjs.AddItem("fault", true);
            fjs.AddItem("held_output", last_output_);
            fjs.AddItem("consecutive_faults", consecutive_faults_);
            fjs.AddTime();
            mqtt_client.publish(std::string("tele/") + settings.sensorName + "/pid", fjs.ToString());
            return ESP_OK;
        }
        consecutive_faults_ = 0;

        // Calculate dt in seconds using esp_timer_get_time().
        uint64_t current_time = esp_timer_get_time(); // in microseconds
        double dt = 0.0;
        if (lastTime == 0) {
            // On first call, use the loop period as a default.
            dt = periodMs_.load() / 1000.0;
        } else {
            dt = (current_time - lastTime) / 1e6;
        }
        lastTime = current_time;

        double output;
        // Pass dt to the compute function.
        auto rjs = pid.compute(reflux_temp, dt, output);
        reflux_cooling_motor.setDutyPercentage(output);
        last_output_ = output;

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
        running_.store(false);
        if (pidTaskHandle_ != nullptr) {
            vTaskDelete(pidTaskHandle_);
            pidTaskHandle_ = nullptr;
        }
    }

    // (Re)start the loop at periodMs, or stop it when periodMs == 0.
    void start(uint32_t periodMs) {
        periodMs_.store(periodMs);
        lastTime = 0;            // reset dt baseline on (re)start
        pid.resetDerivative();   // avoid a derivative spike on the first sample
        running_.store(periodMs != 0);

        if (pidTaskHandle_ == nullptr) {
            // Stack is generous: runPIDControl builds JSON and does an HTTP POST.
            if (xTaskCreate(pidTaskTrampoline, "pid_loop", 4096, this, 5, &pidTaskHandle_) != pdPASS) {
                ESP_LOGE("PIDControlTimer", "Failed to create PID task");
                pidTaskHandle_ = nullptr;
                return;
            }
        }
        // Wake the task so it picks up the new state immediately.
        xTaskNotifyGive(pidTaskHandle_);

        if (periodMs == 0) {
            ESP_LOGI("PIDControlTimer", "PID loop stopped");
        } else {
            ESP_LOGI("PIDControlTimer", "PID loop running with a period of %u ms", (unsigned int)periodMs);
        }
    }

    void resetAll() {
        reflux_temp_sensor.reset();
        boiler_temp_sensor.reset();
    }
};
