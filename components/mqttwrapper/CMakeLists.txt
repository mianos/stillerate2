idf_component_register(SRCS "src/MqttClient.cpp"
                        INCLUDE_DIRS "include"  "${PROJECT_DIR}/main"
                       REQUIRES mqtt
                       nvsstoragemanager
                       json
					   jsonwrapper)
