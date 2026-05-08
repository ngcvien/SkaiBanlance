#ifndef ROBOT_CMDS_H
#define ROBOT_CMDS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Danh sach lenh robot
typedef enum {
    CMD_NONE = 0,
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_STOP
} robot_cmd_t;

// Queue giao tiep giua Voice Task va Command Task
extern QueueHandle_t robot_cmd_queue;

// Khai bao ham
void robot_core_init(void);

#endif