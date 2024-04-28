set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include the ESP-IDF components
idf_component_register(SRCS main.cpp 
                MqttClient.cpp
                INCLUDE_DIRS "."
                REQUIRES
				    driver
                    esp_timer
					mqtt
                    button
                    wifimanager
                    nvsstoragemanager
                    jsonwrapper
                    pidwrapper
                    maxwrapper)
