#pragma once
#include "JsonWrapper.h"
#include "NvsStorageManager.h"
#include <cassert>
#include <iostream>
#include <string>



class PIDController {
public:
    double kp{0.0}, ki{0.0}, kd{0.0}, set_point{0.0};
    double prev_error{0.0}, integral{0.0}, output_min{0.0}, output_max{100.0};
    double integral_limit{100.0}, lag_time_constant{0.0}, sample_time{0.0};
    double prev_output{0.0}, rate_limit{0.0}, derivative_filter_state{0.0};
    NvsStorageManager& nvs;
    std::string nvsKey;

    PIDController(NvsStorageManager& nvsManager, double set_point, double kp, double ki, double kd, double lag_time_constant, double sample_time, double rate_limit, const std::string& nvsKey = "pid_params")
        : kp{kp}, ki{ki}, kd{kd}, set_point{set_point}, lag_time_constant{lag_time_constant}, sample_time{sample_time}, rate_limit{rate_limit}, nvs{nvsManager}, nvsKey{nvsKey} {
        loadParameters();
    }


    JsonWrapper compute(double current_temp, float& output) {
        double error = set_point - current_temp; // Heating-mode PID error

        double original_integral = integral;
        integral += error * sample_time;
        integral = std::clamp(integral, -integral_limit, integral_limit);

        double raw_derivative = (error - prev_error) / sample_time;
        double alpha = sample_time / (lag_time_constant + sample_time);
        derivative_filter_state += alpha * (raw_derivative - derivative_filter_state);

        double heat_output = std::clamp((kp * error) + (ki * integral) + (kd * derivative_filter_state), output_min, output_max);

        if (heat_output == output_min || heat_output == output_max) integral = original_integral;

        if (rate_limit > 0.0) {
            double delta = heat_output - prev_output;
            double max_delta = rate_limit * sample_time;
            if (std::abs(delta) > max_delta) {
                heat_output = prev_output + std::copysign(max_delta, delta);
                heat_output = std::clamp(heat_output, output_min, output_max);
            }
        }

        double cooling_output = std::clamp((output_max - output_min) - heat_output, output_min, output_max);

        output = static_cast<float>(cooling_output);
        prev_error = error;
        prev_output = cooling_output;

        JsonWrapper json;
        json.AddItem("current_temp", current_temp);
        json.AddItem("error", error);
        json.AddItem("heat_output", heat_output);
        json.AddItem("output", cooling_output);
        json.AddItem("integral", integral);
        json.AddItem("filtered_derivative", derivative_filter_state);
        return json;
    }


    void setOutputLimits(double min, double max) { output_min = min; output_max = max; }
    void setIntegralLimit(double limit) { integral_limit = limit; }
    void setRateLimit(double limit) { rate_limit = limit; }

    bool setParametersFromJsonWrapper(const JsonWrapper& json) {
        bool updated = false;
        updated |= json.GetField("kp", kp, true);
        updated |= json.GetField("ki", ki, true);
        updated |= json.GetField("kd", kd, true);
        updated |= json.GetField("set_point", set_point, true);
        updated |= json.GetField("lag_time_constant", lag_time_constant, true);
        updated |= json.GetField("sample_time", sample_time, true);
        updated |= json.GetField("output_min", output_min, true);
        updated |= json.GetField("output_max", output_max, true);
        updated |= json.GetField("integral_limit", integral_limit, true);
        updated |= json.GetField("rate_limit", rate_limit, true);
        if (updated) { saveParameters(); return true; }
        return false;
    }

    void toJsonWrapper(JsonWrapper& json) const {
        json.AddItem("kp", kp);
        json.AddItem("ki", ki);
        json.AddItem("kd", kd);
        json.AddItem("set_point", set_point);
        json.AddItem("lag_time_constant", lag_time_constant);
        json.AddItem("sample_time", sample_time);
        json.AddItem("output_min", output_min);
        json.AddItem("output_max", output_max);
        json.AddItem("integral_limit", integral_limit);
        json.AddItem("rate_limit", rate_limit);
    }

    bool loadParameters() {
        std::string jsonParams;
        if (nvs.retrieve(nvsKey, jsonParams)) {
            JsonWrapper json = JsonWrapper::Parse(jsonParams);
            return setParametersFromJsonWrapper(json);
        }
        return false;
    }

    void saveParameters() {
        JsonWrapper json;
        toJsonWrapper(json);
        nvs.store(nvsKey, json.ToString());
    }
};

