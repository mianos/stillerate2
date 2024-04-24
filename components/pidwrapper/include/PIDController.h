#pragma once

#include "pid_ctrl.h"
#include <cassert>

class PIDController {
public:
    // Constructor that takes initial PID parameters
    PIDController(const pid_ctrl_parameter_t& params) {
        pid_ctrl_config_t config;
        config.init_param = params;

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

private:
    pid_ctrl_block_handle_t pid_handle = nullptr; // Handle to the underlying PID control block
};

// Example usage:
// pid_ctrl_parameter_t params = {1.0, 0.01, 0.001, 100.0, -100.0, 50.0, -50.0, PID_CAL_TYPE_POSITIONAL};
// PIDController pid(params);
// float output;
// if (!pid.compute(0.5, output)) {
//     // handle error
// }
// if (!pid.updateParameters(new_params)) {
//     // handle error
// }
// if (!pid.reset()) {
//     // handle error
// }
