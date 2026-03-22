set(srcs
    main.c
    )

set(requires
    hardware_driver
    robot_core
    )

idf_component_register(SRCS ${srcs}
                       REQUIRES ${requires})