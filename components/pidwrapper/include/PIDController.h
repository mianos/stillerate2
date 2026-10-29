#pragma once
#include <cmath>
#include <mutex>

#include "JsonWrapper.h"
#include "NvsStorageManager.h"
#include "PidCore.h"

// PIDController = PidCore (pure math) + JSON telemetry + NVS persistence +
// thread-safety. The gains live here as the canonical, JSON-mapped values and
// are copied into the core on each compute(). All public methods are guarded by
// a recursive mutex because parameters are mutated from the REST/HTTP task
// while compute() runs in the control-loop task.
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
        if (!applyParameters(json)) {
            return false;
        }
        saveParameters();
        return true;
    }

    bool loadParameters() {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        std::string jsonParams;
        if (!nvs.retrieve(nvsKey, jsonParams)) {
            return false;
        }
        // Apply without saving: what was read is already what NVS holds, so
        // re-storing it on every boot would only wear the flash.
        applyParameters(JsonWrapper::Parse(jsonParams));
        return true;
    }

    void saveParameters() {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        JsonWrapper json;
        toJsonWrapperImpl(json);
        nvs.store(nvsKey, json.ToString());
    }

    // Fail-safe output for a cooling loop: maximum coolant (total reflux).
    double failSafeOutput() const {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        return output_max;
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

    // Reads a numeric field, rejecting non-finite values. JSON can smuggle them
    // in through overflow — cJSON parses 1e400 with strtod, which yields inf —
    // and an inf/nan gain or setpoint would poison every later step and then be
    // persisted to NVS. Returns false (leaving `value` untouched) for a field
    // that is absent, of the wrong type, or not finite.
    static bool getFinite(const JsonWrapper& json, const std::string& key, double& value) {
        double v;
        if (!json.GetField(key, v, true)) {
            return false;
        }
        if (!std::isfinite(v)) {
            ESP_LOGE("pid", "%s = %g is not finite; ignoring", key.c_str(), v);
            return false;
        }
        value = v;
        return true;
    }

    // Applies every valid field of `json`, leaving invalid or absent ones at
    // their previous value. Returns true if at least one parameter was applied.
    // Caller must hold mtx_.
    bool applyParameters(const JsonWrapper& json) {
        bool updated = false;

        // integral_limit bounds the integral state as +/-limit, i.e. it is used
        // as a std::clamp bound: a negative value would invert the bounds
        // (undefined behaviour), so it must be >= 0.
        {
            double v = integral_limit;
            if (getFinite(json, "integral_limit", v)) {
                if (v >= 0.0) {
                    integral_limit = v;
                    updated = true;
                } else {
                    ESP_LOGE("pid", "integral_limit %.4f < 0; ignoring", v);
                }
            }
        }
        // wind_down_threshold is compared against |error|; a negative value
        // would silently disable the integral leak.
        {
            double v = wind_down_threshold;
            if (getFinite(json, "wind_down_threshold", v)) {
                if (v >= 0.0) {
                    wind_down_threshold = v;
                    updated = true;
                } else {
                    ESP_LOGE("pid", "wind_down_threshold %.4f < 0; ignoring", v);
                }
            }
        }
        // integral_unwind_factor is applied as pow(1 - factor, dt); a value
        // outside [0, 1) makes the base <= 0 and yields NaN, poisoning the
        // integral. Reject an out-of-range value (keep the previous one) and log.
        {
            double f;
            if (getFinite(json, "integral_unwind_factor", f)) {
                if (f >= 0.0 && f < 1.0) {
                    integral_unwind_factor = f;
                    updated = true;
                } else {
                    ESP_LOGE("pid", "integral_unwind_factor %.4f out of range [0,1); ignoring", f);
                }
            }
        }
        // derivative_filter is an EMA coefficient with the same half-open range
        // (0 = unfiltered). Reject anything else instead of letting the core
        // silently ignore it, so the reported value is the one in force.
        {
            double f;
            if (getFinite(json, "derivative_filter", f)) {
                if (f >= 0.0 && f < 1.0) {
                    derivative_filter = f;
                    updated = true;
                } else {
                    ESP_LOGE("pid", "derivative_filter %.4f out of range [0,1); ignoring", f);
                }
            }
        }
        // Gains may be negative (reverse-acting loops are legitimate), but never
        // non-finite.
        updated |= getFinite(json, "kd", kd);
        updated |= getFinite(json, "ki", ki);
        updated |= getFinite(json, "kp", kp);
        // Output limits are validated as a pair: the pump accepts 0..100 only
        // (anything else is silently refused), and std::clamp with min > max is
        // undefined behaviour. A bad pair is rejected whole, keeping the old one.
        {
            double lo = output_min, hi = output_max;
            const bool got_lo = getFinite(json, "output_min", lo);
            const bool got_hi = getFinite(json, "output_max", hi);
            if (got_lo || got_hi) {
                if (lo >= 0.0 && hi <= 100.0 && lo <= hi) {
                    output_min = lo;
                    output_max = hi;
                    updated = true;
                } else {
                    ESP_LOGE("pid", "output limits [%.1f, %.1f] invalid (need 0 <= min <= max <= 100); ignoring", lo, hi);
                }
            }
        }
        updated |= getFinite(json, "set_point", set_point);

        return updated;
    }

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
