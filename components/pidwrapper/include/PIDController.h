#pragma once
#include "JsonWrapper.h"
#include "NvsStorageManager.h"

class PIDController {
public:
    double kp{0.0}, ki{0.0}, kd{0.0};
    double output_min{0.0}, output_max{100.0}, dead_zone_threshold{30.0};
    double integral_limit{100.0};
    double set_point{0.0};

    NvsStorageManager& nvs;
    const std::string nvsKey;

private:
    double integral{0.0};
    double previous_error{0.0};
    double autopid_input{0.0};

public:
    PIDController(NvsStorageManager& nvsManager, const std::string& key = "pid_params")
        : nvs{nvsManager}, nvsKey{key} {
        loadParameters();
    }

	JsonWrapper compute(double current_temp, double& output) {
		double error = set_point - current_temp;

		// Deadzone handling for cooling system
		if (error > dead_zone_threshold) {
			// Temperature is below the setpoint - deadzone: no cooling
			output = output_min;
		} else if (error < -dead_zone_threshold) {
			// Temperature is above the setpoint + deadzone: maximum cooling
			output = output_max;
		} else {
			// Temperature is within the deadzone: apply PID control
			double proportional = kp * error;

			// Integral term with windup limits
			integral += error;
			if (integral > integral_limit) integral = integral_limit;
			if (integral < -integral_limit) integral = -integral_limit;

			double derivative = kd * (error - previous_error);
			previous_error = error;

			output = proportional + ki * integral + derivative;
			if (output > output_max) output = output_max;
			if (output < output_min) output = output_min;
		}

		// Build JSON report
		JsonWrapper json;
		toJsonWrapper(json); // Add static parameters

		// Add dynamic parameters (avoid duplicates)
		json.AddItem("current_temp", current_temp);
		json.AddItem("error", error);
		json.AddItem("output", output);
		json.AddItem("previous_error", previous_error);
		json.AddItem("proportional", kp * error);
		json.AddItem("integral", integral);
		json.AddItem("derivative", kd * (error - previous_error));

		return json;
	}


    void toJsonWrapper(JsonWrapper& json) const {
        json.AddItem("dead_zone_threshold", dead_zone_threshold);
        json.AddItem("integral_limit", integral_limit);
        json.AddItem("kd", kd);
        json.AddItem("ki", ki);
        json.AddItem("kp", kp);
        json.AddItem("output_max", output_max);
        json.AddItem("output_min", output_min);
        json.AddItem("set_point", set_point);
    }

    bool setParametersFromJsonWrapper(const JsonWrapper& json) {
        bool updated = false;

        updated |= json.GetField("dead_zone_threshold", dead_zone_threshold, true);
        updated |= json.GetField("integral_limit", integral_limit, true);
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
        integral = 0.0;
        previous_error = 0.0;
        autopid_input = 0.0;
    }
};
