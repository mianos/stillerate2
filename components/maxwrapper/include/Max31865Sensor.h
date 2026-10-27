#pragma once

#include <cstdint>
#include <limits>

#include <esp_timer.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <max31865.h>

class SPIBusInitializer {
public:
    static void initialize(int mosi, int miso, int clk) {
        static bool isInitialized = false;
        if (!isInitialized) {
            spi_bus_config_t busCfg = {};
            busCfg.mosi_io_num = mosi;
            busCfg.miso_io_num = miso;
            busCfg.sclk_io_num = clk;
            busCfg.quadwp_io_num = -1;
            busCfg.quadhd_io_num = -1;
            busCfg.max_transfer_sz = 0;
            busCfg.flags = 0;
            ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &busCfg, SPI_DMA_CH_AUTO));
            isInitialized = true;
        }
    }
};

class Max31865Sensor {
public:
    static constexpr float kDefaultRref = 401.5f;  // measured by the HP3478A 4 wire
    static constexpr float kMinPlausibleC = -50.0f;
    static constexpr float kMaxPlausibleC = 300.0f;

    struct Diagnostics {
        uint16_t raw = 0;
        bool     fault = false;
        uint8_t  faultStatus = 0;
        float    ohms = 0.0f;
        float    temperatureC = std::numeric_limits<float>::quiet_NaN();
        float    rRef = 0.0f;
        float    offsetC = 0.0f;
        esp_err_t err = ESP_OK;
    };

	// TODO: put these values in a Kconfig
    Max31865Sensor(gpio_num_t cs_pin, const char* name = "rtd",
                   int mosi_io = 18, int miso_io = 20, int clk_io = 19)
        : name_(name), csPin(cs_pin), mosiIO(mosi_io), misoIO(miso_io), clkIO(clk_io) {
        mutex_ = xSemaphoreCreateMutex();
        dev = {};
        // The driver's "DIN43760" is the US alpha 0.00392 curve (reads ~1.5 C
        // low at 78 C); "ITS90" is IEC 60751 alpha 0.00385, which these probes are.
        dev.standard = MAX31865_ITS90;
        dev.r_ref = kDefaultRref;
        dev.rtd_nominal = 100.0f;
        config = {};
        config.v_bias = true;
        config.filter = MAX31865_FILTER_50HZ;
        config.mode = MAX31865_MODE_SINGLE;
        config.connection = MAX31865_3WIRE;

        SPIBusInitializer::initialize(mosiIO, misoIO, clkIO);
        initializeSensor();
    }

    void setCalibration(float r_ref, float offset_c) {
        Lock lock(mutex_);
        if (r_ref > 0.0f) dev.r_ref = r_ref;
        offsetC_ = offset_c;
        ESP_LOGI(name_, "calibration r_ref=%.2f ohm offset=%.2f C", dev.r_ref, offsetC_);
    }

    float measure() {
        Lock lock(mutex_);
        float temperature;
        esp_err_t result = max31865_measure(&dev, &temperature);
        if (result != ESP_OK) {
            reportAndClearFault(result);
            return std::numeric_limits<float>::quiet_NaN();
        }
        // A shorted input reads raw 0 (-242 C) without setting the fault bit.
        if (temperature < kMinPlausibleC || temperature > kMaxPlausibleC) {
            ESP_LOGE(name_, "implausible reading %.2f C", temperature);
            return std::numeric_limits<float>::quiet_NaN();
        }
        return temperature + offsetC_;
    }

    Diagnostics diagnostics() {
        Lock lock(mutex_);
        Diagnostics d;
        d.rRef = dev.r_ref;
        d.offsetC = offsetC_;
        d.err = max31865_start_measurement(&dev);
        if (d.err != ESP_OK) return d;
        vTaskDelay(pdMS_TO_TICKS(70));
        d.err = max31865_read_raw(&dev, &d.raw, &d.fault);
        if (d.err != ESP_OK) return d;
        d.ohms = d.raw * dev.r_ref / 32768.0f;
        max31865_get_fault_status(&dev, &d.faultStatus);
        if (d.fault) {
            reportAndClearFault(ESP_FAIL);
            return d;
        }
        float temperature;
        if (max31865_read_temperature(&dev, &temperature) == ESP_OK) {
            d.temperatureC = temperature + offsetC_;
        }
        return d;
    }

    // max31865_reset zeroes the config (bias off, 2-wire), so reapply ours.
    esp_err_t reset() {
        Lock lock(mutex_);
        esp_err_t err = max31865_reset(&dev);
        if (err != ESP_OK) {
            ESP_LOGE(name_, "reset failed: %s", esp_err_to_name(err));
            return err;
        }
        err = max31865_set_config(&dev, &config);
        ESP_LOGI(name_, "reset %s", err == ESP_OK ? "ok" : esp_err_to_name(err));
        return err;
    }

private:
    struct Lock {
        explicit Lock(SemaphoreHandle_t m) : m_(m) { xSemaphoreTake(m_, portMAX_DELAY); }
        ~Lock() { xSemaphoreGive(m_); }
        SemaphoreHandle_t m_;
    };

    const char* name_;
    SemaphoreHandle_t mutex_ = nullptr;
    float offsetC_ = 0.0f;
    max31865_t dev;
    max31865_config_t config;
    gpio_num_t csPin;
    int mosiIO;
    int misoIO;
    int clkIO;

    void initializeSensor() {
        ESP_ERROR_CHECK(max31865_init_desc(&dev, SPI2_HOST, MAX31865_MAX_CLOCK_SPEED_HZ, csPin));
        ESP_ERROR_CHECK(max31865_set_config(&dev, &config));
    }

    // The fault bit latches; without clearing it one glitch kills the channel until reboot.
    void reportAndClearFault(esp_err_t result) {
        uint8_t status = 0;
        if (result == ESP_FAIL && max31865_get_fault_status(&dev, &status) == ESP_OK) {
            ESP_LOGE(name_, "RTD fault 0x%02x:%s%s%s%s%s%s", status,
                     (status & 0x80) ? " rtd-high-threshold" : "",
                     (status & 0x40) ? " rtd-low-threshold" : "",
                     (status & 0x20) ? " refin-high(open-rtd)" : "",
                     (status & 0x10) ? " refin-low(force-open)" : "",
                     (status & 0x08) ? " rtdin-low(force-open)" : "",
                     (status & 0x04) ? " over/under-voltage" : "");
            max31865_clear_fault_status(&dev);
            max31865_set_config(&dev, &config);
        } else {
            ESP_LOGE(name_, "measure failed: %s", esp_err_to_name(result));
        }
    }
};
