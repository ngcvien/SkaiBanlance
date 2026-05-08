#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include <esp_err.h>

// Hàm khởi tạo I2C và đánh thức MPU6050
esp_err_t mpu6050_init(void);

// Hàm đọc dữ liệu và chạy thuật toán lọc. 
// Trả về góc nghiêng mượt mà (Pitch) tính bằng Độ.
float mpu6050_get_smoothed_pitch(void);

#endif