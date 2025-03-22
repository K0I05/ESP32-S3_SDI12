idf_component_register(
    SRCS sdi12_master.c
    INCLUDE_DIRS include
    REQUIRES esp_driver_uart esp_driver_gpio esp_timer
)
