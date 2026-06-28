#include "robot_control.h"
#include "robot_cmds.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>

// A small tilt drives the robot while the angle PID remains in control.
#define DRIVE_TARGET_ANGLE_DEG  3.0f
#define TURN_SPEED_STEPS_S      900.0f

static portMUX_TYPE motion_lock = portMUX_INITIALIZER_UNLOCKED;
static robot_motion_setpoint_t motion_setpoint = {0};

static void set_motion(float target_angle_deg, float turn_speed_steps_s) {
    taskENTER_CRITICAL(&motion_lock);
    motion_setpoint.target_angle_deg = target_angle_deg;
    motion_setpoint.turn_speed_steps_s = turn_speed_steps_s;
    taskEXIT_CRITICAL(&motion_lock);
}

void robot_control_get_setpoint(robot_motion_setpoint_t *setpoint) {
    if (setpoint == NULL) {
        return;
    }

    taskENTER_CRITICAL(&motion_lock);
    *setpoint = motion_setpoint;
    taskEXIT_CRITICAL(&motion_lock);
}

void robot_control_emergency_stop(void) {
    set_motion(0.0f, 0.0f);
}

static void robot_control_task(void *pvParameters) {
    robot_cmd_t current_cmd;

    while (1) {
        if (xQueueReceive(robot_cmd_queue, &current_cmd, portMAX_DELAY) == pdPASS) {
            printf(">>> NHAN LENH: %d\n", current_cmd);
            switch (current_cmd) {
                case CMD_FORWARD:
                    printf(">>> HANH DONG: TIEN\n");
                    set_motion(DRIVE_TARGET_ANGLE_DEG, 0.0f);
                    break;
                case CMD_BACKWARD:
                    printf(">>> HANH DONG: LUI\n");
                    set_motion(-DRIVE_TARGET_ANGLE_DEG, 0.0f);
                    break;
                case CMD_TURN_LEFT:
                    printf(">>> HANH DONG: QUAY TRAI\n");
                    set_motion(0.0f, TURN_SPEED_STEPS_S);
                    break;
                case CMD_TURN_RIGHT:
                    printf(">>> HANH DONG: QUAY PHAI\n");
                    set_motion(0.0f, -TURN_SPEED_STEPS_S);
                    break;
                case CMD_STOP:
                    printf(">>> HANH DONG: DUNG LAI\n");
                    set_motion(0.0f, 0.0f);
                    break;
                default:
                    break;
            }
        }
    }
}

void robot_control_task_start(void) {
    if (robot_cmd_queue == NULL) {
        printf(">>> TASK: Chua co queue lenh, khong the khoi chay!\n");
        return;
    }

    if (xTaskCreate(robot_control_task, "robot_ctrl_task", 4096, NULL, 5, NULL) == pdPASS) {
        printf(">>> TASK: Dieu khien robot da khoi chay!\n");
    } else {
        printf(">>> TASK: Khong the tao robot_ctrl_task!\n");
    }
}
