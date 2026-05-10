#include "stepper_driver.h"
#include "hardware_config.h"
#include "driver/gpio.h"
#include "driver/ledc.h" // Thư viện điều khiển xung phần cứng của ESP-IDF
#include <math.h>

void stepper_driver_init(void) {
    // 1. Cấu hình chân DIR (Hướng quay) thành Output bình thường
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<STEPPER_LEFT_DIR) | (1ULL<<STEPPER_RIGHT_DIR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // 2. Cấu hình Bộ đếm giờ (Timer) của LEDC
    // Timer 0 cho Bánh Trái
    ledc_timer_config_t timer_left = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT, // Độ phân giải xung 10-bit (0-1023)
        .freq_hz          = 100,               // Tần số khởi tạo tạm thời
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_left);

    // Timer 1 cho Bánh Phải (Dùng Timer riêng để 2 bánh có thể chạy tốc độ khác nhau)
    ledc_timer_config_t timer_right = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_1,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .freq_hz          = 100,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_right);

    // 3. Cấu hình Kênh phát xung (Nối Timer thẳng ra chân STEP phần cứng)
    ledc_channel_config_t channel_left = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = STEPPER_LEFT_STEP,
        .duty           = 0, // Duty = 0 nghĩa là không phát xung (Dừng)
        .hpoint         = 0
    };
    ledc_channel_config(&channel_left);

    ledc_channel_config_t channel_right = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = STEPPER_RIGHT_STEP,
        .duty           = 0, 
        .hpoint         = 0
    };
    ledc_channel_config(&channel_right);
}

void stepper_set_speed(float speed_left, float speed_right) {
    // --- XỬ LÝ BÁNH TRÁI ---
    if (fabs(speed_left) < 45.0f) {
        // Nếu tốc độ quá nhỏ, tắt phát xung để dừng động cơ
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    } else {
        // Cài đặt chiều quay
        gpio_set_level(STEPPER_LEFT_DIR, (speed_left > 0) ? 0 : 1);
        
        // Nạp tần số mới trực tiếp vào phần cứng (Số bước / giây)
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, (uint32_t)fabs(speed_left));
        
        // Kích hoạt phát xung với độ rộng 50% (512 / 1023)
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }

    // --- XỬ LÝ BÁNH PHẢI ---
    if (fabs(speed_right) < 45.0f) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    } else {
        // Thường bánh phải lắp ngược hướng vật lý với bánh trái, nên logic DIR sẽ đảo lại
        gpio_set_level(STEPPER_RIGHT_DIR, (speed_right > 0) ? 1 : 0);
        
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_1, (uint32_t)fabs(speed_right));
        
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 512);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
}