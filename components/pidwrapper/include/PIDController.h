#pragma once
#include "JsonWrapper.h"
#include "NvsStorageManager.h"
#include <cassert>
#include <iostream>
#include <string>
#include <cmath>


class PIDController {
public:
    double kp{0.0}, ki{0.0}, kd{0.0}, set_point{0.0};
    double prev_error{0.0}, integral{0.0}, output_min{0.0}, output_max{100.0};
    double integral_limit{100.0}, lag_time_constant{0.0}, sample_time{0.0};
    double prev_output{0.0}, rate_limit{0.0}, derivative_filter_state{0.0};
	double dead_zone_threshold{30.0};
	double prev_adjusted_error{0.0};
    NvsStorageManager& nvs;
    std::string nvsKey;

    PIDController(NvsStorageManager& nvsManager, double set_point, double kp, double ki, double kd, double lag_time_constant, double sample_time, double rate_limit, const std::string& nvsKey = "pid_params")
        : kp{kp}, ki{ki}, kd{kd}, set_point{set_point}, lag_time_constant{lag_time_constant}, sample_time{sample_time}, rate_limit{rate_limit}, nvs{nvsManager}, nvsKey{nvsKey} {
        loadParameters();
    }


	JsonWrapper compute(double current_temp, double& output) {
		// Calculate the raw error: set_point - current_temp
		double error = set_point - current_temp;
		
		// Compute the adjusted error using the configurable dead zone threshold.
		// This logic implements:
		//   - If error > dead_zone_threshold (vapor far below setpoint): no cooling (adjusted_error = 0).
		//   - If 0 <= error <= dead_zone_threshold: adjusted_error = dead_zone_threshold - error.
		//   - If error < 0 (above setpoint): adjusted_error = dead_zone_threshold + |error| to drive aggressive cooling.
		double adjusted_error = 0.0;
		if (error > dead_zone_threshold) {
			adjusted_error = 0.0;
		} else if (error >= 0 && error <= dead_zone_threshold) {
			adjusted_error = dead_zone_threshold - error;
		} else { // error < 0
			adjusted_error = dead_zone_threshold + fabs(error);
		}
		
		// **Proportional Term** using adjusted_error
		double proportional = kp * adjusted_error;
		
		// **Integral Term (with Anti-Windup)**
		integral += ki * adjusted_error * sample_time;
		integral = std::clamp(integral, -integral_limit, integral_limit);
		
		// **Derivative Term** using adjusted_error
		double derivative = (adjusted_error - prev_adjusted_error) / sample_time;
		derivative_filter_state += (derivative - derivative_filter_state) * (sample_time / lag_time_constant);
		double derivative_output = kd * derivative_filter_state;
		
		// **Compute Final Output**
		double cooling_output = proportional + integral + derivative_output;
		
		// **Check for Integral Windup (after computing output)**
		if (cooling_output >= output_max || cooling_output <= output_min) {
			integral -= ki * adjusted_error * sample_time;  // Undo accumulation if output is saturated
		}
		
		// **Apply Rate Limiting First**
		double max_delta = rate_limit * sample_time;
		cooling_output = std::clamp(cooling_output, prev_output - max_delta, prev_output + max_delta);
		
		// **Clamp Output to Allowed Range**
		cooling_output = std::clamp(cooling_output, output_min, output_max);
		
		// **Store Values for Next Cycle**
		prev_error = error;
		prev_adjusted_error = adjusted_error;
		prev_output = cooling_output;
		
		// **Prepare JSON Debugging Output**
		JsonWrapper json;
		json.AddItem("current_temp", current_temp);
		json.AddItem("error", error);
		json.AddItem("adjusted_error", adjusted_error);
		json.AddItem("proportional", proportional);
		json.AddItem("integral", integral);
		json.AddItem("derivative", derivative_output);
		json.AddItem("output", cooling_output);
		json.AddItem("kp", kp);
		json.AddItem("ki", ki);
		json.AddItem("kd", kd);
		json.AddItem("dead_zone_threshold", dead_zone_threshold);
		
		// **Set Output Reference**
		output = cooling_output;
		
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
        updated |= json.GetField("dead_zone_threshold", dead_zone_threshold, true);
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
        json.AddItem("dead_zone_threshold", dead_zone_threshold);
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

