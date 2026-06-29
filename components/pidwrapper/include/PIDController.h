#pragma once
#include <mutex>

#include "JsonWrapper.h"
#include "NvsStorageManager.h"
#include "PidCore.h"

// PIDController = PidCore (pure math) + JSON telemetry + NVS persistence +
// thread-safety. The gains live here as the canonical, JSON-mapped values and
// are copied into the core on each compute(). All public methods are guarded by
// a recursive mutex because parameters are mutated from the MQTT task while
// compute() runs in the control-loop/timer task.
class PIDController {
public:
    double kp{0.0}, ki{0.0}, kd{0.0};
    double output_min{0.0}, output_max{100.0};
    double integral_limit{100.0}, integral_unwind_factor{0.1};
    double wind_down_threshold{4.0};
    double derivative_filter{0.0};   // EMA coeff in [0,1) for the D term; 0 = off
    double set_point{0.0};

    NvsStorageManager& nvs;
    const std::string nvsKey;

    PIDController(NvsStorageManager& nvsManager, const std::string& key = "pid_params")
        : nvs{nvsManager}, nvsKey{key} {
        loadParameters();
    }

    // Run one control step. Returns a JSON diagnostic report and writes the
    // commanded output (0..output_max) into `output`.
    JsonWrapper compute(double current_temp, double dt, double& output) {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        syncGainsToCore();
        PidResult res = core_.compute(current_temp, dt);
        output = res.output;

        JsonWrapper json;
        toJsonWrapperImpl(json);
        json.AddItem("current_temp", current_temp);
        json.AddItem("error", res.error);
        json.AddItem("output", res.output);
        json.AddItem("P", res.P);
        json.AddItem("I", res.I);
        json.AddItem("D", res.D);
        json.AddItem("integral", res.integral);
        json.AddItem("dt", res.dt);
        json.AddItem("saturated", res.saturated);
        return json;
    }

    void toJsonWrapper(JsonWrapper& json) const {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        toJsonWrapperImpl(json);
    }

    bool setParametersFromJsonWrapper(const JsonWrapper& json) {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        bool updated = false;
        updated |= json.GetField("integral_limit", integral_limit, true);
        updated |= json.GetField("wind_down_threshold", wind_down_threshold, true);
        updated |= json.GetField("integral_unwind_factor", integral_unwind_factor, true);
        updated |= json.GetField("derivative_filter", derivative_filter, true);
        updated |= json.GetField("kd", kd, true);
        updated |= json.GetField("ki", ki, true);
        updated |= json.GetField("kp", kp, true);
        updated |= json.GetField("output_max", output_max, true);
        updated |= json.GetField("output_min", output_min, true);
        updated |= json.GetField("set_point", set_point, true);

        if (updated) {
            saveParameters();
        }
        return updated;
    }

    bool loadParameters() {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        std::string jsonParams;
        if (nvs.retrieve(nvsKey, jsonParams)) {
            JsonWrapper json = JsonWrapper::Parse(jsonParams);
            return setParametersFromJsonWrapper(json);
        }
        return false;
    }

    void saveParameters() {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        JsonWrapper json;
        toJsonWrapperImpl(json);
        nvs.store(nvsKey, json.ToString());
    }

    // Full reset: clears integral and derivative history.
    void reset() {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        ESP_LOGI("pid", "reset PID");
        core_.reset();
    }

    // Clear only derivative history (preserves integral) — call when the control
    // loop is (re)started so the first sample does not produce a derivative spike.
    void resetDerivative() {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        core_.resetDerivative();
    }

private:
    mutable std::recursive_mutex mtx_;
    PidCore core_;

    void syncGainsToCore() {
        core_.kp = kp;
        core_.ki = ki;
        core_.kd = kd;
        core_.output_min = output_min;
        core_.output_max = output_max;
        core_.integral_limit = integral_limit;
        core_.integral_unwind_factor = integral_unwind_factor;
        core_.wind_down_threshold = wind_down_threshold;
        core_.derivative_filter = derivative_filter;
        core_.set_point = set_point;
    }

    void toJsonWrapperImpl(JsonWrapper& json) const {
        json.AddItem("integral_limit", integral_limit);
        json.AddItem("wind_down_threshold", wind_down_threshold);
        json.AddItem("integral_unwind_factor", integral_unwind_factor);
        json.AddItem("derivative_filter", derivative_filter);
        json.AddItem("kd", kd);
        json.AddItem("ki", ki);
        json.AddItem("kp", kp);
        json.AddItem("output_max", output_max);
        json.AddItem("output_min", output_min);
        json.AddItem("set_point", set_point);
    }
};
