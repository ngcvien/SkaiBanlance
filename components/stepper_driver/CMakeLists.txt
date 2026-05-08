idf_component_register(
    SRCS "stepper_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer hardware_config
)