#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <stdint.h>

// Khởi tạo các chân GPIO và Timer cho động cơ
void stepper_driver_init(void);

// Hàm cài đặt tốc độ cho 2 bánh xe. 
// Đơn vị: Số bước mỗi giây (Steps per second). Giá trị âm (-) để chạy lùi.
void stepper_set_speed(float speed_left, float speed_right);

#endif