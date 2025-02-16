#pragma once
#include "JsonWrapper.h"
#include "NvsStorageManager.h"
#include <cassert>
#include <iostream>

class PIDController {
public:
    double kp;
    double ki;
    double kd;
    double set_point;
    double prev_error;
    double integral;
    double lag_time_constant;
    double sample_time;
    NvsStorageManager& nvs;
    std::string nvsKey;

    PIDController(NvsStorageManager& nvsManager, double set_point, double kp, double ki, double kd, double lag_time_constant, double sample_time, const std::string& nvsKey = "pid_params")
        : kp(kp), ki(ki), kd(kd), set_point(set_point), prev_error(0.0), integral(0.0), lag_time_constant(lag_time_constant), sample_time(sample_time), nvs(nvsManager), nvsKey(nvsKey) {
        loadParameters();
    }

    JsonWrapper compute(double input_error, float& output) {  // Changed output to float reference
        integral += input_error * sample_time;
        double derivative = (input_error - prev_error) / sample_time;
        double lagged_derivative = (derivative + prev_error * lag_time_constant / sample_time) / (1.0 + lag_time_constant / sample_time);
        double pid_output = kp * input_error + ki * integral + kd * lagged_derivative;
        output = static_cast<float>(pid_output);  // Cast output to float
        prev_error = input_error;

        JsonWrapper json;
        json.AddItem("input_error", input_error);
        json.AddItem("output", output);
        return json;
    }

    bool updateParameters(double new_kp, double new_ki, double new_kd, double new_lag_time_constant, double new_sample_time) {
        kp = new_kp;
        ki = new_ki;
        kd = new_kd;
        lag_time_constant = new_lag_time_constant;
        sample_time = new_sample_time;
        saveParameters();
        return true;
    }

    bool setParametersFromJsonWrapper(const JsonWrapper& json) {
        bool updated = false;
        updated |= json.GetField("kp", kp, true);
        updated |= json.GetField("ki", ki, true);
        updated |= json.GetField("kd", kd, true);
        updated |= json.GetField("set_point", set_point, true);
        updated |= json.GetField("lag_time_constant", lag_time_constant, true);
        updated |= json.GetField("sample_time", sample_time, true);
        if (updated) {
            saveParameters();
            return true;
        }
        return false;
    }

    bool reset() {
        prev_error = 0.0;
        integral = 0.0;
        return true;
    }

    void toJsonWrapper(JsonWrapper& json) const {
        json.AddItem("kp", kp);
        json.AddItem("ki", ki);
        json.AddItem("kd", kd);
        json.AddItem("set_point", set_point);
        json.AddItem("lag_time_constant", lag_time_constant);
        json.AddItem("sample_time", sample_time);
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
