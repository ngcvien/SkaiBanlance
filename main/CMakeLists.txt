set(srcs
    main.c
    )

set(requires
    hardware_driver
    robot_core
    stepper_driver
    hardware_config
    mpu6050_driver
    pid_controller
    )

idf_component_register(SRCS ${srcs}
                       REQUIRES ${requires} ble_server hardware_config  nvs_flash)