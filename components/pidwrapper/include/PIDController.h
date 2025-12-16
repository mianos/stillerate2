#pragma once
#include "JsonWrapper.h"
#include "NvsStorageManager.h"

class PIDController {
public:
    double kp{0.0}, ki{0.0}, kd{0.0};
    double output_min{0.0}, output_max{100.0};
    double integral_limit{100.0}, integral_unwind_factor{0.1}; // New unwind factor
	double wind_down_threshold{4.0};
    double set_point{0.0};
	double dead_zone{0.1};

    NvsStorageManager& nvs;
    const std::string nvsKey;
private:
    double integral{0.0};
    double previous_error{0.0};

public:





JsonWrapper compute(double current_temp, double dt, double &output) {
    double error = current_temp - set_point;  // Fix error sign

    // Proportional term
    double P = kp * error;

    // Integral term update (only accumulate if output is within limits)
    if (output < output_max && output > output_min) {
        integral += error * dt;
    }

    // Apply wind-down if error is small
    if (fabs(error) < wind_down_threshold) {
        integral *= (1.0 - integral_unwind_factor);
    }

    // Clamp integral to prevent excessive accumulation
    if (integral > integral_limit) integral = integral_limit;
    if (integral < -integral_limit) integral = -integral_limit;

    double I = ki * integral;

    // Derivative term (rate of change)
    double D = 0.0;
    if (dt > 0) {
        D = kd * ((error - previous_error) / dt);
    }
    previous_error = error;

    // Compute final control output
    double u = P + I + D;

    // Enforce output limits
    if (u < output_min) u = output_min;
    if (u > output_max) u = output_max;

    output = u;

    // Build JSON diagnostic report
    JsonWrapper json;
    toJsonWrapper(json);
    json.AddItem("current_temp", current_temp);
    json.AddItem("error", error);
    json.AddItem("output", output);
    json.AddItem("P", P);
    json.AddItem("I", I);
    json.AddItem("D", D);
    json.AddItem("integral", integral);
    json.AddItem("dt", dt);
    return json;
}



    PIDController(NvsStorageManager& nvsManager, const std::string& key = "pid_params")
        : nvs{nvsManager}, nvsKey{key} {
        loadParameters();
    }



    void toJsonWrapper(JsonWrapper& json) const {
        json.AddItem("integral_limit", integral_limit);
        json.AddItem("wind_down_threshold", wind_down_threshold);
        json.AddItem("integral_unwind_factor", integral_unwind_factor); // New parameter
        json.AddItem("kd", kd);
        json.AddItem("ki", ki);
        json.AddItem("kp", kp);
        json.AddItem("output_max", output_max);
        json.AddItem("output_min", output_min);
        json.AddItem("set_point", set_point);
    }

    bool setParametersFromJsonWrapper(const JsonWrapper& json) {
        bool updated = false;

        updated |= json.GetField("integral_limit", integral_limit, true);
        updated |= json.GetField("wind_down_threshold", wind_down_threshold, true);
        updated |= json.GetField("integral_unwind_factor", integral_unwind_factor, true); // New parameter
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

    void reset() {
		ESP_LOGI("pid", "reset PID");

        integral = 0.0;
		previous_error = 0.0;
    }
};

