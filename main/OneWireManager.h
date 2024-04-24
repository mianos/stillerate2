#pragma once

#include "esp_log.h"
#include "driver/gpio.h"
#include "onewire_bus.h"
#include "ds18b20.h"

constexpr int EXAMPLE_ONEWIRE_BUS_GPIO = 4; // Example GPIO number for OneWire bus
constexpr int EXAMPLE_ONEWIRE_MAX_DS18B20 = 10; // Maximum number of DS18B20 devices

class OneWireManager {
public:
    OneWireManager();
    ~OneWireManager();

    void installBus();
    void searchDevices();
    void setResolutionAllDevices();
    void readTemperatures();

private:
    onewire_bus_handle_t bus;
    ds18b20_device_handle_t ds18b20s[EXAMPLE_ONEWIRE_MAX_DS18B20];
    int ds18b20_device_num;
};
