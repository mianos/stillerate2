#pragma once
#include "JsonWrapper.h"
#include "NvsStorageManager.h"
#include <cassert>
#include <iostream>



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

    JsonWrapper compute(double input_error, float& output) {
        integral += input_error * sample_time;
        integral = std::clamp(integral, -integral_limit, integral_limit);

        double raw_derivative = (input_error - prev_error) / sample_time;
        double alpha = sample_time / (lag_time_constant + sample_time);
        derivative_filter_state += alpha * (raw_derivative - derivative_filter_state);
        double filtered_derivative = derivative_filter_state;

        double pid_output = output_max - (kp * input_error + ki * integral + kd * filtered_derivative);
        pid_output = std::clamp(pid_output, output_min, output_max);

        JsonWrapper json;
        if (rate_limit > 0.0) {
            double delta = pid_output - prev_output;
            json.AddItem("delta", delta);
            if (std::abs(delta) > rate_limit * sample_time) {
                pid_output = prev_output + std::copysign(rate_limit * sample_time, delta);
            }
        }

        output = static_cast<float>(pid_output);
        prev_error = input_error;
        prev_output = pid_output;

        json.AddItem("input_error", input_error);
        json.AddItem("output", output);
        json.AddItem("filtered_derivative", filtered_derivative);
        json.AddItem("derivative_filter_state", derivative_filter_state);
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

