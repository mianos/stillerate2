#pragma once
#include "JsonWrapper.h"
#include "NvsStorageManager.h"
#include "AutoPID-for-ESP-IDF.h"

class PIDController {
public:
    double kp{0.0}, ki{0.0}, kd{0.0}, set_point{0.0};
    double output_min{0.0}, output_max{100.0}, dead_zone_threshold{30.0};
    double integral_limit{100.0}, rate_limit{0.0};
    NvsStorageManager& nvs;
    const std::string nvsKey;

private:
    AutoPID autopid;            // AutoPID instance
    double autopid_input;       // Proxy for AutoPID input
    double autopid_setpoint;    // Proxy for AutoPID setpoint
    double autopid_output;      // Proxy for AutoPID output
    double prev_output{0.0};    // For rate limiting
    unsigned long last_time{0}; // For rate limiting

public:
    PIDController(NvsStorageManager& nvsManager, const std::string& key = "pid_params")
        : nvs{nvsManager}, nvsKey{key},
          autopid(&autopid_input, &autopid_setpoint, &autopid_output,
                  output_min, output_max, kp, ki, kd) {
        loadParameters();
        syncAutoPIDParams(); // Initialize AutoPID with current params
    }

    JsonWrapper compute(double current_temp, double& output) {
        // Update AutoPID input/setpoint with dead zone logic
        autopid_input = current_temp;
        autopid_setpoint = set_point;
        
        // AutoPID handles bang-bang (dead zone) and PID
        autopid.run();

#if 0
        // Rate limiting (optional)
        unsigned long now = esp_timer_get_time() / 1000;
        double dt = (now - last_time) / 1000.0;
        if (rate_limit > 0 && dt > 0) {
            double max_change = rate_limit * dt;
            autopid_output = std::clamp(autopid_output,
                                      prev_output - max_change,
                                      prev_output + max_change);
        }
        prev_output = autopid_output;
        last_time = now;

        // Clamp integral (optional)
        if (autopid.getIntegral() > integral_limit) autopid.setIntegral(integral_limit);
        if (autopid.getIntegral() < -integral_limit) autopid.setIntegral(-integral_limit);
#endif
        // Set output
        output = autopid_output;

        // Build JSON report
        double error = set_point - current_temp;
        JsonWrapper json;
        json.AddItem("current_temp", current_temp);
        json.AddItem("error", error);
        json.AddItem("adjusted_error", error); // AutoPID handles dead zone internally
        json.AddItem("proportional", kp * error);
        json.AddItem("integral", autopid.getIntegral());
        json.AddItem("derivative", kd * (error - (set_point - autopid_input))); 
        json.AddItem("output", output);
        return json;
    }

	void set_sample_time(double seconds) {
		autopid.setTimeStep(seconds);
	}

    void toJsonWrapper(JsonWrapper& json) const {
        json.AddItem("dead_zone_threshold", dead_zone_threshold);
        json.AddItem("integral_limit", integral_limit);
        json.AddItem("kd", kd);
        json.AddItem("ki", ki);
        json.AddItem("kp", kp);
        json.AddItem("output_max", output_max);
        json.AddItem("output_min", output_min);
        json.AddItem("rate_limit", rate_limit);
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
        updated |= json.GetField("rate_limit", rate_limit, true);
        updated |= json.GetField("set_point", set_point, true);

        if (updated) {
            syncAutoPIDParams();
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

private:
    void syncAutoPIDParams() {
        autopid.setGains(kp, ki, kd);
        autopid.setBangBang(dead_zone_threshold);
        autopid.setOutputRange(output_min, output_max);
    }


    // Keep existing toJsonWrapper/loadParameters/saveParameters
};
