idf_component_register(
    SRCS "command_manager.c" "robot_control_task.c"
    INCLUDE_DIRS "include" 
    REQUIRES freertos driver hardware_config
)
