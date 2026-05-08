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