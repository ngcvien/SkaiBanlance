idf_component_register(
    SRCS "mpu6050_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer hardware_driver hardware_config
)