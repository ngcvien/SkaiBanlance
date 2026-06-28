#include <stdio.h>
#include "robot_cmds.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "hardware_config.h"

#define ACTIVITY_LED_PULSE_MS  200

static TimerHandle_t activity_led_timer = NULL;

QueueHandle_t robot_cmd_queue = NULL;

static void activity_led_timer_callback(TimerHandle_t timer) {
    (void)timer;
    gpio_set_level(VOICE_CMD_LED_GPIO, 0);
}

void robot_activity_led_pulse(void) {
    if (activity_led_timer == NULL) {
        return;
    }

    gpio_set_level(VOICE_CMD_LED_GPIO, 1);
    if (xTimerReset(activity_led_timer, 0) != pdPASS) {
        gpio_set_level(VOICE_CMD_LED_GPIO, 0);
    }
}

void robot_core_init(void) {
    gpio_config_t led_config = {
        .pin_bit_mask = 1ULL << VOICE_CMD_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (gpio_config(&led_config) != ESP_OK) {
        printf("Loi: Khong the khoi tao LED GPIO%d!\n", VOICE_CMD_LED_GPIO);
    } else {
        gpio_set_level(VOICE_CMD_LED_GPIO, 0);
        activity_led_timer = xTimerCreate(
            "voice_cmd_led",
            pdMS_TO_TICKS(ACTIVITY_LED_PULSE_MS),
            pdFALSE,
            NULL,
            activity_led_timer_callback);
        if (activity_led_timer == NULL) {
            printf("Loi: Khong the tao timer cho LED!\n");
        }
    }

    // Only the newest command matters if producers are faster than the consumer.
    robot_cmd_queue = xQueueCreate(1, sizeof(robot_cmd_t));
    if (robot_cmd_queue == NULL) {
        printf("Loi: Khong the tao queue!\n");
    }
}

bool robot_cmd_send(robot_cmd_t cmd) {
    if (robot_cmd_queue == NULL || cmd <= CMD_NONE || cmd > CMD_STOP) {
        return false;
    }

    if (xQueueOverwrite(robot_cmd_queue, &cmd) != pdPASS) {
        return false;
    }

    robot_activity_led_pulse();
    return true;
}
