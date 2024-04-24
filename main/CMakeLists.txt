set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include the ESP-IDF components
idf_component_register(SRCS main.cpp 
                MqttClient.cpp
                OneWireManager.cpp
                INCLUDE_DIRS "."
                REQUIRES
				    driver
                    esp_timer
					mqtt
                    button
                    wifimanager
                    nvsstoragemanager
                    jsonwrapper
                    ESP32-MAX31865
                    espressif__onewire_bus
                    pidwrapper)
