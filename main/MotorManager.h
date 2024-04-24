#pragma once

#include "driver/ledc.h"
#include "esp_err.h"

class MotorController {
private:
    ledc_channel_config_t ledc_channel;
    ledc_timer_config_t ledc_timer;

public:
    MotorController(int gpio_num, ledc_timer_t timer_num, ledc_channel_t channel_num, ledc_mode_t speed_mode = LEDC_LOW_SPEED_MODE, int frequency = 5000, int duty = 4096, ledc_timer_bit_t duty_resolution = LEDC_TIMER_13_BIT) {
        // Timer configuration
        ledc_timer.speed_mode = speed_mode;
        ledc_timer.timer_num = timer_num;
        ledc_timer.duty_resolution = duty_resolution;
        ledc_timer.freq_hz = frequency;
        ledc_timer.clk_cfg = LEDC_AUTO_CLK;

        // Channel configuration
        ledc_channel.speed_mode = speed_mode;
        ledc_channel.channel = channel_num;
        ledc_channel.timer_sel = timer_num;
        ledc_channel.intr_type = LEDC_INTR_DISABLE;
        ledc_channel.gpio_num = gpio_num;
        ledc_channel.duty = 0;  // Start with duty cycle 0
        ledc_channel.hpoint = 0;

        // Initialize the LEDC peripheral with these settings
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

        // Set initial duty cycle if needed
        setDuty(duty);
    }

    void setDuty(int duty) {
        ESP_ERROR_CHECK(ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel));
    }
};

