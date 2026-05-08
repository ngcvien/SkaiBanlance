#include "robot_control.h"
#include "robot_cmds.h"
#include "stepper_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>

// Bê nguyên nội dung Task từ main.c sang đây
static void robot_control_task(void *pvParameters) {
    robot_cmd_t current_cmd;
    const float SPEED_STRAIGHT = 800.0f;
    const float SPEED_TURN     = 400.0f;

    while (1) {
        if (xQueueReceive(robot_cmd_queue, &current_cmd, portMAX_DELAY) == pdPASS) {
            printf("[Task ID: %p] >>> NHAN LENH: %d\n", xTaskGetCurrentTaskHandle(), current_cmd);
            switch (current_cmd) {
                case CMD_FORWARD:
                    printf(">>> HÀNH ĐỘNG: TIẾN\n");
                    stepper_set_speed(SPEED_STRAIGHT, SPEED_STRAIGHT);
                    break;
                case CMD_BACKWARD:
                    printf(">>> HÀNH ĐỘNG: LÙI\n");
                    stepper_set_speed(-SPEED_STRAIGHT, -SPEED_STRAIGHT);
                    break;
                case CMD_TURN_LEFT:
                    printf(">>> HÀNH ĐỘNG: QUAY TRÁI\n");
                    stepper_set_speed(-SPEED_TURN, SPEED_TURN); 
                    break;
                case CMD_TURN_RIGHT:
                    printf(">>> HÀNH ĐỘNG: QUAY PHẢI\n");
                    stepper_set_speed(SPEED_TURN, -SPEED_TURN);
                    break;
                case CMD_STOP:
                    printf(">>> HÀNH ĐỘNG: DỪNG LẠI\n");
                    stepper_set_speed(0, 0);
                    break;
                default:
                    break;
            }
        }
    }
}

// Hàm này sẽ public ra ngoài để main.c gọi
void robot_control_task_start(void) {
    // Tạo Task chạy ngầm. (Tên Task, RAM 4KB, Tham số NULL, Độ ưu tiên 5)
    xTaskCreate(robot_control_task, "robot_ctrl_task", 4096, NULL, 5, NULL);
    printf(">>> TASK: Giao tiep va Dieu khien dong co da duoc khoi chay!\n");
}