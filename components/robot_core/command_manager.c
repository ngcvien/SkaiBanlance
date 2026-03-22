#include <stdio.h>
#include "robot_cmds.h"

QueueHandle_t robot_cmd_queue = NULL;

void robot_core_init(void) {
    // Tao queue chua toi da 10 lenh
    robot_cmd_queue = xQueueCreate(10, sizeof(robot_cmd_t));
    if (robot_cmd_queue == NULL) {
        printf("Loi: Khong the tao queue!\n");
    }
}

void command_manager_task(void *pvParameters) {
    robot_cmd_t cmd;

    while (1) {
        // Cho nhan lenh tu queue
        if (xQueueReceive(robot_cmd_queue, &cmd, portMAX_DELAY)) {
            switch (cmd) {
                case CMD_FORWARD:
                    printf(">>> ROBOT: DI THANG\n");
                    break;
                case CMD_BACKWARD:
                    printf(">>> ROBOT: DI LUI\n");
                    break;
                case CMD_TURN_LEFT:
                    printf(">>> ROBOT: RE TRAI\n");
                    break;
                case CMD_TURN_RIGHT:
                    printf(">>> ROBOT: RE PHAI\n");
                    break;
                case CMD_STOP:
                    printf(">>> ROBOT: DUNG LAI\n");
                    break;
                default:
                    break;
            }
        }
    }
}