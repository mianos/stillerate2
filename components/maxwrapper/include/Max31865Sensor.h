#pragma once

#include <esp_timer.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/spi_master.h>
#include <max31865.h>

constexpr char Tag[] = "max31865-example";

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
	// TODO: put these values in a Kconfig
    Max31865Sensor(gpio_num_t cs_pin, int mosi_io = 18, int miso_io = 20, int clk_io = 19)
        : csPin(cs_pin), mosiIO(mosi_io), misoIO(miso_io), clkIO(clk_io) {
        dev = {};
        dev.standard = MAX31865_DIN43760;
        dev.r_ref = 401.5f;	// measured by the HP3478A 4 wire
        dev.rtd_nominal = 100.0f;
        config = {};
        config.v_bias = true;
        config.filter = MAX31865_FILTER_50HZ;
        config.mode = MAX31865_MODE_SINGLE;
        config.connection = MAX31865_3WIRE;

        SPIBusInitializer::initialize(mosiIO, misoIO, clkIO);
        initializeSensor();
    }

    float measure() {
        float temperature;
        esp_err_t result = max31865_measure(&dev, &temperature);
        if (result != ESP_OK) {
            ESP_LOGE(Tag, "Failed to measure: %d (%s)", result, esp_err_to_name(result));
            return -1.0f; // Use -1.0f to indicate an error condition
        }
        return temperature;
    }

	esp_err_t reset() {
		return max31865_reset(&dev);
	}

private:
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
};
