#include "OneWireManager.h"

OneWireManager::OneWireManager() : bus(nullptr), ds18b20_device_num(0) {}

OneWireManager::~OneWireManager() {
	onewire_bus_del(bus);
}

void OneWireManager::installBus() {
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = EXAMPLE_ONEWIRE_BUS_GPIO,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // Configure max received bytes as per your requirements
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));
    ESP_LOGI("OneWireManager", "1-Wire bus installed on GPIO%d", EXAMPLE_ONEWIRE_BUS_GPIO);
}

void OneWireManager::searchDevices() {
    onewire_device_iter_handle_t iter = nullptr;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI("OneWireManager", "Device iterator created, start searching...");

    while ((search_result = onewire_device_iter_get_next(iter, &next_onewire_device)) == ESP_OK) {
        ds18b20_config_t ds_cfg = {};
        if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20s[ds18b20_device_num]) == ESP_OK) {
            ESP_LOGI("OneWireManager", "Found a DS18B20[%d], address: %016llX", ds18b20_device_num, next_onewire_device.address);
            ds18b20_device_num++;
            if (ds18b20_device_num >= EXAMPLE_ONEWIRE_MAX_DS18B20) {
                ESP_LOGI("OneWireManager", "Max DS18B20 number reached, stop searching...");
                break;
            }
        }
    }
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_LOGI("OneWireManager", "Searching done, %d DS18B20 device(s) found", ds18b20_device_num);
}

void OneWireManager::setResolutionAllDevices() {
    for (int i = 0; i < ds18b20_device_num; i++) {
        ESP_ERROR_CHECK(ds18b20_set_resolution(ds18b20s[i], DS18B20_RESOLUTION_12B));
    }
}

void OneWireManager::readTemperatures() {
    float temperature;
    for (int i = 0; i < ds18b20_device_num; i++) {
        ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(ds18b20s[i]));
        ESP_ERROR_CHECK(ds18b20_get_temperature(ds18b20s[i], &temperature));
        ESP_LOGI("OneWireManager", "Temperature read from DS18B20[%d]: %.2fC", i, temperature);
    }
}
