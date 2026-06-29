idf_component_register(
    SRCS max31865.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_spi esp_driver_gpio log
)
