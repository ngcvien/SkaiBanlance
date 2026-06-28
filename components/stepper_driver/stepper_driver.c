#include "stepper_driver.h"
#include "hardware_config.h"
#include "driver/gpio.h"
#include "driver/gptimer.h" // Sử dụng thư viện Hardware Timer
#include "esp_attr.h"
#include <math.h>

// ==========================================
// CẤU HÌNH ĐẢO CHIỀU ĐỘNG CƠ TẠI ĐÂY
// Nếu robot ngã tới mà bánh xe lùi lại, hãy đổi 1 thành -1 ở bánh xe tương ứng
#define LEFT_REVERSE  -1
#define RIGHT_REVERSE 1 
// ==========================================

// Biến toàn cục đếm số bước
static volatile int32_t step_count_left = 0;
static volatile int32_t step_count_right = 0;

// Chiều chuyển động logic của xe (1: Tiến, -1: Lùi).
// Chiều này độc lập với mức DIR vật lý vì hai motor được lắp đối xứng.
static volatile int8_t dir_left = 1;
static volatile int8_t dir_right = 1;

// Trạng thái chân STEP (Bật/Tắt)
static volatile bool pin_state_left = false;
static volatile bool pin_state_right = false;

// Trình quản lý Hardware Timer
static gptimer_handle_t timer_left = NULL;
static gptimer_handle_t timer_right = NULL;
static bool timer_left_running = false;
static bool timer_right_running = false;

// Hàm ngắt (ISR) cho Bánh Trái
static bool IRAM_ATTR timer_left_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    if (!pin_state_left) {
        step_count_left += dir_left; // Cộng hoặc trừ bước đi
        gpio_set_level(STEPPER_LEFT_STEP, 1);
        pin_state_left = true;
    } else {
        gpio_set_level(STEPPER_LEFT_STEP, 0);
        pin_state_left = false;
    }
    return false;
}

// Hàm ngắt (ISR) cho Bánh Phải
static bool IRAM_ATTR timer_right_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    if (!pin_state_right) {
        step_count_right += dir_right;
        gpio_set_level(STEPPER_RIGHT_STEP, 1);
        pin_state_right = true;
    } else {
        gpio_set_level(STEPPER_RIGHT_STEP, 0);
        pin_state_right = false;
    }
    return false;
}

void stepper_driver_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<STEPPER_LEFT_STEP) | (1ULL<<STEPPER_LEFT_DIR) | 
                        (1ULL<<STEPPER_RIGHT_STEP) | (1ULL<<STEPPER_RIGHT_DIR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Cấu hình Timer đếm với tần số 1MHz (1 micro-giây mỗi nhịp)
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, 
    };
    gptimer_new_timer(&timer_config, &timer_left);
    gptimer_new_timer(&timer_config, &timer_right);

    gptimer_event_callbacks_t cbs_left = { .on_alarm = timer_left_isr };
    gptimer_register_event_callbacks(timer_left, &cbs_left, NULL);
    gptimer_enable(timer_left);

    gptimer_event_callbacks_t cbs_right = { .on_alarm = timer_right_isr };
    gptimer_register_event_callbacks(timer_right, &cbs_right, NULL);
    gptimer_enable(timer_right);
}

void stepper_set_speed(float speed_left, float speed_right) {
    // Lưu chiều chuyển động logic trước khi đảo chiều motor phần cứng.
    // Nhờ vậy khi xe chạy thẳng, bộ đếm hai bánh luôn cùng dấu.
    const int8_t logical_dir_left = (speed_left >= 0.0f) ? 1 : -1;
    const int8_t logical_dir_right = (speed_right >= 0.0f) ? 1 : -1;

    // Nhân với hệ số đảo chiều phần cứng
    speed_left *= LEFT_REVERSE;
    speed_right *= RIGHT_REVERSE;

    // Xử lý Bánh Trái
    if (fabs(speed_left) < 10.0f) {
        if (timer_left_running) {
            gptimer_stop(timer_left);
            timer_left_running = false;
        }
    } else {
        dir_left = logical_dir_left;
        gpio_set_level(STEPPER_LEFT_DIR, (speed_left > 0) ? 1 : 0);
        
        // Tính toán chu kỳ ngắt (Micro-giây)
        uint64_t alarm_val = (uint64_t)(1000000.0f / (fabs(speed_left) * 2.0f));
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = alarm_val,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true,
        };
        
        gptimer_set_alarm_action(timer_left, &alarm_config);
        if (!timer_left_running) {
            gptimer_start(timer_left);
            timer_left_running = true;
        }
    }

    // Xử lý Bánh Phải
    if (fabs(speed_right) < 10.0f) {
        if (timer_right_running) {
            gptimer_stop(timer_right);
            timer_right_running = false;
        }
    } else {
        dir_right = logical_dir_right;
        gpio_set_level(STEPPER_RIGHT_DIR, (speed_right > 0) ? 1 : 0);
        
        uint64_t alarm_val = (uint64_t)(1000000.0f / (fabs(speed_right) * 2.0f));
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = alarm_val,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true,
        };
        
        gptimer_set_alarm_action(timer_right, &alarm_config);
        if (!timer_right_running) {
            gptimer_start(timer_right);
            timer_right_running = true;
        }
    }
}

int32_t stepper_get_left_step(void) { return step_count_left; }
int32_t stepper_get_right_step(void) { return step_count_right; }

void stepper_reset_step(void) {
    step_count_left = 0;
    step_count_right = 0;
}
