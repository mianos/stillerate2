#pragma once
#include <atomic>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "MqttClient.h"
#include "StillContext.h"
#include "Emulation.h"
#include "SettingsManager.h"
#include "PIDController.h"
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
    Max31865Sensor& middle_temp_sensor;   // mid-column RTD; still calibrated/reset, but not the control PV
    RESTMotorController& reflux_cooling_motor;
    Max31865Sensor& head_temp_sensor;     // control PV: vapour temperature at the head (top)
    Emulation& emu;

    // Set by start() (MQTT task), consumed by the PID task: the next sample uses
    // the loop period as dt instead of the stale lastTime. An atomic flag rather
    // than zeroing the 64-bit lastTime across tasks, which can tear on RV32.
    std::atomic<bool> resyncDt_{true};

    // Touched only by the PID task, so no synchronisation needed.
    uint64_t lastTime = 0;         // in microseconds
    double   last_output_ = 0.0;
    bool     have_output_ = false;  // a good sample has been controlled on
    int      consecutive_faults_ = 0;
    bool     pump_ok_ = true;
    bool     pump_stalled_ = false;
    bool     drivingPump_ = false;    // loop has commanded the pump since it last started
    bool     wasRunning_ = false;
    int      pump_rpm_ = -1;          // -1: not reported
    TickType_t lastRunTicks_ = 0;   // how long the previous runPIDControl took

    // Sensor-fault policy. A transient bad read holds the last good output so a
    // single glitch doesn't jolt the pump. Past kFaultHoldSamples consecutive
    // faults — or with no good sample yet — the loop is flying blind, so it
    // fails safe to maximum coolant (total reflux: nothing leaves the column).
    // Holding 0 here would switch the dephlegmator pump OFF with the boiler on.
    static constexpr int kFaultHoldSamples = 3;
    static constexpr double kPumpOffDuty = 0.5;  // pump controller disables the drive at or below this

    // Plausible column-temperature window (degrees C). Anything outside this, or
    // non-finite (the sensor returns NaN on a read failure), is treated as a
    // fault. Column temps live in roughly 0..100 C; the margin covers transients.
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
                // Done here, not in the stop handler, so a sample already in
                // progress can't turn the pump back on after it.
                if (drivingPump_) {
                    commandPump(0.0);
                    drivingPump_ = false;
                    ESP_LOGI("PIDControlTimer", "Loop stopped; reflux pump off");
                }
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // sleep until start()
                continue;
            }
            const uint32_t p = periodMs_.load();
            // Wait out the remainder of the period: the previous run's time
            // (sensor read, publishes, pump POST) is subtracted so the sample
            // interval stays close to the period rather than period + run time.
            const TickType_t period = pdMS_TO_TICKS(p ? p : 1000);
            const TickType_t wait = period > lastRunTicks_ ? period - lastRunTicks_ : 1;
            // Interruptible wait; nonzero return means we were notified (state
            // changed) — re-evaluate without running this cycle.
            if (ulTaskNotifyTake(pdTRUE, wait) != 0) {
                continue;
            }
            if (!running_.load()) {
                continue;
            }
            const TickType_t started = xTaskGetTickCount();
            drivingPump_ = true;
            (void)runPIDControl();
            lastRunTicks_ = xTaskGetTickCount() - started;
        }
    }

    // Sends the duty to the pump and tracks whether it got through. A pump that
    // stops answering is logged once on the transition (not every sample) and
    // reported in the telemetry as pump_ok, so a loop driving a dead pump is
    // visible instead of silent.
    void commandPump(double duty) {
        const bool ok = reflux_cooling_motor.setDutyPercentage(duty);
        updateStall(duty, ok);
        if (ok != pump_ok_) {
            if (ok) {
                ESP_LOGI("PIDControlTimer", "Reflux pump reachable again");
            } else {
                ESP_LOGE("PIDControlTimer", "Reflux pump not accepting commands; loop output %.1f%% not applied", duty);
            }
            pump_ok_ = ok;
        }
    }

    // pump_ok can't see a stall: the controller answers either way. The rpm in
    // its reply predates the command, so only judge a duty held for a full sample.
    void updateStall(double duty, bool ok) {
        const bool running = ok && duty > kPumpOffDuty;
        const auto rpm = reflux_cooling_motor.lastRpm();
        pump_rpm_ = rpm.value_or(-1);
        const bool stalled = running && wasRunning_ && rpm && *rpm == 0;
        wasRunning_ = running;
        if (stalled != pump_stalled_) {
            if (stalled) {
                ESP_LOGE("PIDControlTimer", "Reflux pump stalled at %.1f%%", duty);
            } else {
                ESP_LOGI("PIDControlTimer", "Reflux pump no longer stalled");
            }
            pump_stalled_ = stalled;
        }
    }

    void addPumpFields(JsonWrapper& js) const {
        js.AddItem("pump_ok", pump_ok_);
        js.AddItem("pump_stalled", pump_stalled_);
        if (pump_rpm_ >= 0) js.AddItem("pump_rpm", pump_rpm_);
    }

    esp_err_t runPIDControl() {
        float head_temp;
        bool valid = true;

        if (emu.enabled) {
            head_temp = emu.temp;
        } else {
            head_temp = head_temp_sensor.measure();
            valid = std::isfinite(head_temp) &&
                    head_temp > kMinValidTemp && head_temp < kMaxValidTemp;
        }

        // On a sensor fault, do NOT run the PID (a garbage reading would corrupt
        // the integral and spike the derivative). See kFaultHoldSamples.
        if (!valid) {
            ++consecutive_faults_;
            const bool fail_safe = !have_output_ || consecutive_faults_ > kFaultHoldSamples;
            const double fault_output = fail_safe ? pid.failSafeOutput() : last_output_;
            ESP_LOGW("PIDControlTimer",
                     "Invalid head temperature (%.2f); %s output %.1f%% (fault #%d)",
                     head_temp, fail_safe ? "fail-safe" : "holding", fault_output, consecutive_faults_);
            commandPump(fault_output);
            JsonWrapper fjs;
            fjs.AddItem("fault", true);
            fjs.AddItem("fail_safe", fail_safe);
            fjs.AddItem("held_output", fault_output);
            fjs.AddItem("consecutive_faults", consecutive_faults_);
            addPumpFields(fjs);
            fjs.AddTime();
            mqtt_client.publish(std::string("tele/") + settings.sensorName + "/pid", fjs.ToString());
            return ESP_OK;
        }
        consecutive_faults_ = 0;

        // dt in seconds. After a (re)start the loop period stands in for the
        // elapsed time, since lastTime is from before the stop.
        const uint64_t current_time = esp_timer_get_time(); // in microseconds
        const double dt = resyncDt_.exchange(false)
            ? periodMs_.load() / 1000.0
            : (current_time - lastTime) / 1e6;
        lastTime = current_time;

        double output;
        auto rjs = pid.compute(head_temp, dt, output);
        commandPump(output);
        last_output_ = output;
        have_output_ = true;

        if (emu.enabled) {
            rjs.AddItem("emulation", true);
        }
        addPumpFields(rjs);
        rjs.AddTime();
        mqtt_client.publish(std::string("tele/") + settings.sensorName + "/pid", rjs.ToString());
        return ESP_OK;
    }

public:
    PIDControlTimer(PIDController& pid,
                      MqttClient& mqtt_client,
                      SettingsManager &settings,
                      Max31865Sensor& middle_temp,
                      RESTMotorController& reflux_cooling_motor,
                      Max31865Sensor& head_temp,
                      Emulation& emu)
        : pid(pid), mqtt_client(mqtt_client),
          settings(settings),
          middle_temp_sensor(middle_temp),
          reflux_cooling_motor(reflux_cooling_motor),
          head_temp_sensor(head_temp),
          emu(emu) {
        applySensorCalibration();
    }

    void applySensorCalibration() {
        middle_temp_sensor.setCalibration(settings.middleRref, settings.middleOffset);
        head_temp_sensor.setCalibration(settings.headRref, settings.headOffset);
    }

    ~PIDControlTimer() {
        running_.store(false);
        if (pidTaskHandle_ != nullptr) {
            vTaskDelete(pidTaskHandle_);
            pidTaskHandle_ = nullptr;
        }
    }

    // (Re)start the loop at periodMs, or stop it when periodMs == 0.
    uint32_t periodMs() const { return periodMs_.load(); }

    void start(uint32_t periodMs) {
        periodMs_.store(periodMs);
        resyncDt_.store(true);   // reset dt baseline on (re)start
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
        middle_temp_sensor.reset();
        head_temp_sensor.reset();
    }
};
