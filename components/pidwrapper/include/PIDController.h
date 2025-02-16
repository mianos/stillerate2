#pragma once
#include "JsonWrapper.h"
#include "NvsStorageManager.h"
#include <cassert>
#include <iostream>

class PIDController {
public:
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double set_point{0.0};
    double prev_error{0.0};
    double integral{0.0};
    double lag_time_constant{0.0};
    double sample_time{0.0};
    double output_min{0.0};
    double output_max{100.0};
    double integral_limit{100.0};
    NvsStorageManager& nvs;
    std::string nvsKey;

    PIDController(NvsStorageManager& nvsManager, double set_point, double kp, double ki, double kd, double lag_time_constant, double sample_time, const std::string& nvsKey = "pid_params")
        : kp{kp}, ki{ki}, kd{kd}, set_point{set_point}, lag_time_constant{lag_time_constant}, sample_time{sample_time}, nvs{nvsManager}, nvsKey{nvsKey} {
        loadParameters();
    }

    JsonWrapper compute(double input_error, float& output) {
        integral += input_error * sample_time;
        if (integral > integral_limit) integral = integral_limit;
        if (integral < -integral_limit) integral = -integral_limit;

        double derivative = (input_error - prev_error) / sample_time;
        double lagged_derivative = (derivative + prev_error * lag_time_constant / sample_time) / (1.0 + lag_time_constant / sample_time);
        double pid_output = kp * input_error + ki * integral + kd * lagged_derivative;

        if (pid_output > output_max) pid_output = output_max;
        if (pid_output < output_min) pid_output = output_min;

        output = static_cast<float>(pid_output);
        prev_error = input_error;

        JsonWrapper json;
        json.AddItem("input_error", input_error);
        json.AddItem("output", output);
        return json;
    }

    void setOutputLimits(double min, double max) { output_min = min; output_max = max; }
    void setIntegralLimit(double limit) { integral_limit = limit; }

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

