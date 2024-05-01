#pragma once

#include "pid_ctrl.h"
#include "JsonWrapper.h"
#include "NvsStorageManager.h"
#include <cassert>
#include <iostream>

#pragma once

#include "pid_ctrl.h"
#include "JsonWrapper.h"
#include "NvsStorageManager.h"
#include <cassert>
#include <iostream>

class PIDController {
public:
    // Constructor that takes a reference to an NvsStorageManager and initial PID parameters
    PIDController(NvsStorageManager& nvsManager, double set_point, const pid_ctrl_parameter_t& initialParams, const std::string& nvsKey = "pid_params")
        : nvs(nvsManager),
          pid_handle(nullptr),
          currentParams(initialParams),
          nvsKey(nvsKey),
		  set_point(set_point)

    {
        // Initialize PID control block
        pid_ctrl_config_t config = {};
        config.init_param = currentParams; // Ensure currentParams is fully initialized
        esp_err_t result = pid_new_control_block(&config, &pid_handle);
        assert(result == ESP_OK && "Failed to create PID control block");

        // Load parameters or use initial parameters if not available
        if (!loadParameters()) {
            updateParameters(initialParams);
        }
    }

    ~PIDController() {
        if (pid_handle) {
            esp_err_t result = pid_del_control_block(pid_handle);
            assert(result == ESP_OK && "Failed to delete PID control block");
        }
    }

    // Load parameters from NVS
    bool loadParameters() {
        std::string jsonParams;
        if (nvs.retrieve(nvsKey, jsonParams)) {
            return setParametersFromJson(jsonParams);
        }
        return false;
    }

    // Save parameters to NVS
    void saveParameters() {
        std::string jsonParams = getParametersAsJson();
        nvs.store(nvsKey, jsonParams);
    }

    // Update the PID parameters
    bool updateParameters(const pid_ctrl_parameter_t& params) {
        currentParams = params; // Update local copy
        esp_err_t result = pid_update_parameters(pid_handle, &params);
        if (result == ESP_OK) {
            saveParameters(); // Save updated parameters to NVS
            return true;
        }
        return false;
    }
    // Compute the PID output given an input error
	JsonWrapper compute(float input_error, float& output) {
        assert(pid_handle != nullptr && "PID handle is not initialized");
        JsonWrapper json;
        pid_compute(pid_handle, input_error, &output);
        //esp_err_t result = pid_compute(pid_handle, input_error, &output);
		// json.AddItem("cstatus", result);
		json.AddItem("input", input_error);
		json.AddItem("output", output);
		return json;
    }

    // Reset the PID controller state
    bool reset() {
        assert(pid_handle != nullptr && "PID handle is not initialized");
        esp_err_t result = pid_reset_ctrl_block(pid_handle);
        return result == ESP_OK;
    }

    // Serialize current PID parameters to JSON
    std::string getParametersAsJson() const {
        JsonWrapper json;
        json.AddItem("kp", currentParams.kp);
        json.AddItem("ki", currentParams.ki);
        json.AddItem("kd", currentParams.kd);
        json.AddItem("max_output", currentParams.max_output);
        json.AddItem("min_output", currentParams.min_output);
        json.AddItem("max_integral", currentParams.max_integral);
        json.AddItem("min_integral", currentParams.min_integral);
        json.AddItem("set_point", set_point);
        return json.ToString();
    }

    // Update parameters from JSON string
    bool setParametersFromJson(const std::string& jsonString) {
		return setParametersFromJsonWrapper(JsonWrapper::Parse(jsonString));
	}

    bool setParametersFromJsonWrapper(const JsonWrapper& json) {
        pid_ctrl_parameter_t newParams = currentParams;  // Start with current parameters
        bool updated = false;

        updated |= json.GetField("kp", newParams.kp, true);
        updated |= json.GetField("ki", newParams.ki, true);
        updated |= json.GetField("kd", newParams.kd, true);
        updated |= json.GetField("max_output", newParams.max_output, true);
        updated |= json.GetField("min_output", newParams.min_output, true);
        updated |= json.GetField("max_integral", newParams.max_integral, true);
        updated |= json.GetField("min_integral", newParams.min_integral, true);
        updated |= json.GetField("set_point", set_point, true);

        if (updated) {
			ESP_LOGI("PID", "Updating pid param");
            return updateParameters(newParams);
        }
        return false;
    }

private:
    NvsStorageManager& nvs; // Reference to NVS storage manager
    pid_ctrl_block_handle_t pid_handle; // Handle to the underlying PID control block
    pid_ctrl_parameter_t currentParams; // Current PID parameters
    std::string nvsKey; // NVS key for storing and retrieving PID parameters
public:
	double set_point;
};
