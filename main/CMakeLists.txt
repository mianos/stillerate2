set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include the ESP-IDF components
idf_component_register(SRCS main.cpp 
                INCLUDE_DIRS "."
                REQUIRES
				    esp_driver_gpio
                    esp_timer
					esp_https_ota esp_system bootloader_support
					espressif__mqtt
					espressif__cjson
                    mqttwrapper
                    button
                    wifimanager
                    nvsstoragemanager
                    jsonwrapper
                    pidwrapper
                    maxwrapper
                    httpclient)
