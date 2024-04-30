#pragma once

#include "pid_ctrl.h"
#include "JsonWrapper.h"
#include <cassert>
#include <iostream>

class PIDController {
public:
    // Constructor that takes initial PID parameters
    PIDController(const pid_ctrl_parameter_t& params)
        : currentParams(params) {
        pid_ctrl_config_t config;
        config.init_param = currentParams;

        esp_err_t result = pid_new_control_block(&config, &pid_handle);
        // Assert to ensure PID control block is created successfully
        assert(result == ESP_OK && "Failed to create PID control block");
    }

    // Destructor to clean up the PID control block
    ~PIDController() {
        if (pid_handle) {
            esp_err_t result = pid_del_control_block(pid_handle);
            assert(result == ESP_OK && "Failed to delete PID control block");
        }
    }

    // Update the PID parameters
    bool updateParameters(const pid_ctrl_parameter_t& params) {
        currentParams = params; // Update local copy
        esp_err_t result = pid_update_parameters(pid_handle, &params);
        return result == ESP_OK;
    }

    // Compute the PID output given an input error
    bool compute(float input_error, float& output) {
        esp_err_t result = pid_compute(pid_handle, input_error, &output);
        return result == ESP_OK;
    }

    // Reset the PID controller state
    bool reset() {
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
        return json.ToString();
    }

    // Update parameters from JSON string
    bool setParametersFromJson(const std::string& jsonString) {
        JsonWrapper json = JsonWrapper::Parse(jsonString);
        pid_ctrl_parameter_t newParams = currentParams;  // Start with current parameters
        bool updated = false;

        updated |= json.GetField("kp", newParams.kp);
        updated |= json.GetField("ki", newParams.ki);
        updated |= json.GetField("kd", newParams.kd);
        updated |= json.GetField("max_output", newParams.max_output);
        updated |= json.GetField("min_output", newParams.min_output);
        updated |= json.GetField("max_integral", newParams.max_integral);
        updated |= json.GetField("min_integral", newParams.min_integral);

        if (updated) {
            return updateParameters(newParams);
        }
        return false;
    }

private:
    pid_ctrl_parameter_t currentParams;
    pid_ctrl_block_handle_t pid_handle = nullptr; // Handle to the underlying PID control block
};
