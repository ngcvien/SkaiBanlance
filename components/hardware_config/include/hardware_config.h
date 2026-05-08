#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "driver/gpio.h"

/* =========================================================
 * SƠ ĐỒ KẾT NỐI (PIN MAP) CHO ESP32-S3 ROBOT
 * ========================================================= */

/* 1. MPU6050 - CẢM BIẾN GÓC NGHIÊNG (GIAO TIẾP I2C) */
#define I2C_MASTER_SDA_IO   (GPIO_NUM_4)
#define I2C_MASTER_SCL_IO   (GPIO_NUM_5)

/* 2. MICRO DỮ LIỆU ÂM THANH (GIAO TIẾP I2S) */
#define GPIO_I2S_LRCK       (GPIO_NUM_11)
#define GPIO_I2S_MCLK       (GPIO_NUM_NC)
#define GPIO_I2S_SCLK       (GPIO_NUM_12)
#define GPIO_I2S_SDIN       (GPIO_NUM_10)

/* 3. ĐỘNG CƠ TRÁI (MOTOR LEFT) */
#define STEPPER_LEFT_STEP   (GPIO_NUM_16)
#define STEPPER_LEFT_DIR    (GPIO_NUM_17)

/* 4. ĐỘNG CƠ PHẢI (MOTOR RIGHT) */
#define STEPPER_RIGHT_STEP  (GPIO_NUM_14)
#define STEPPER_RIGHT_DIR   (GPIO_NUM_15)

#endif // HARDWARE_CONFIG_H