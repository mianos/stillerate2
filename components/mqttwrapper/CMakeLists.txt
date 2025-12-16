idf_component_register(SRCS "src/MqttClient.cpp"
                        INCLUDE_DIRS "include"
                       REQUIRES mqtt
                       nvsstoragemanager
                       json
					   jsonwrapper)
