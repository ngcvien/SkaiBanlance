#include "mpu6050_driver.h"
#include "hardware_config.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>

#define I2C_PORT_NUM    I2C_NUM_0
#define MPU6050_ADDR    0x68    // Địa chỉ I2C mặc định của MPU6050
#define RAD_TO_DEG      57.29577951f
#define ALPHA           0.98f   // Hệ số lọc bù (98% tin Gyro, 2% tin Accel)

static const char *TAG = "MPU_DRIVER";
static volatile float current_pitch = 0.0f;
static uint64_t last_time_us = 0;

// Hàm hỗ trợ ghi 1 byte vào thanh ghi của MPU6050
static esp_err_t mpu_write_reg(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_PORT_NUM, MPU6050_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_init(void) {
    // 1. Cấu hình chân I2C theo bản đồ phần cứng
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000, // Tốc độ I2C 400kHz
    };
    i2c_param_config(I2C_PORT_NUM, &conf);
    i2c_driver_install(I2C_PORT_NUM, conf.mode, 0, 0, 0);

    // 2. Đánh thức MPU6050 (Thanh ghi 0x6B -> 0x00)
    if (mpu_write_reg(0x6B, 0x00) != ESP_OK) {
        ESP_LOGE(TAG, "Khong tim thay MPU6050! Kiem tra lai day dien.");
        return ESP_FAIL;
    }

    // 3. BẬT BỘ LỌC PHẦN CỨNG DLPF (CỰC KỲ QUAN TRỌNG CHỐNG RUNG)
    // Thanh ghi 0x1A: Ghi giá trị 0x04 -> Băng thông 20Hz (Lọc sạch nhiễu động cơ)
    // Nếu robot vẫn rung, đổi thành 0x05 (Băng thông 10Hz)
    mpu_write_reg(0x1A, 0x04);

    // 4. Cấu hình độ nhạy Gyroscope (+/- 250 độ/giây)
    mpu_write_reg(0x1B, 0x00);
    
    // 5. Cấu hình độ nhạy Accelerometer (+/- 2g)
    mpu_write_reg(0x1C, 0x00);

    ESP_LOGI(TAG, "Khoi tao MPU6050 va DLPF thanh cong!");
    return ESP_OK;
}

float mpu6050_get_smoothed_pitch(void) {
    uint8_t data[14];
    
    // Đọc 14 byte dữ liệu liên tục bắt đầu từ thanh ghi 0x3B (ACCEL_XOUT_H)
    // Chứa: 6 byte Accel, 2 byte Nhiệt độ, 6 byte Gyro
    uint8_t reg = 0x3B;
    if (i2c_master_write_read_device(I2C_PORT_NUM, MPU6050_ADDR, &reg, 1, data, 14, 1000 / portTICK_PERIOD_MS) != ESP_OK) {
        return current_pitch; // Nếu lỗi I2C, trả về góc cũ
    }

    // Gộp 2 byte thành số nguyên 16-bit
    int16_t acc_x = (data[0] << 8) | data[1];
    int16_t acc_y = (data[2] << 8) | data[3];
    int16_t acc_z = (data[4] << 8) | data[5];
    int16_t gyro_x = (data[8] << 8) | data[9]; // Tốc độ xoay quanh trục X
    // int16_t gyro_y = (data[10] << 8) | data[11];
    // int16_t gyro_z = (data[12] << 8) | data[13];

    // 1. Tính delta thời gian (dt)
    uint64_t current_time_us = esp_timer_get_time();
    if (last_time_us == 0) {
        last_time_us = current_time_us;
        return 0.0f;
    }
    float dt = (current_time_us - last_time_us) / 1000000.0f;
    last_time_us = current_time_us;

    // 2. TÍNH GÓC GIA TỐC (ACCEL PITCH)
    // Lưu ý: Công thức atan2 này phụ thuộc vào chiều bạn gắn MPU6050.
    // Giả sử robot ngã tới lui theo trục X (xoay quanh trục Y), công thức là:
    float accel_pitch = atan2((float)acc_x, sqrt((float)acc_y * acc_y + (float)acc_z * acc_z)) * RAD_TO_DEG;

    // Nếu bạn gắn chip sao cho nó xoay ngã tới lui quanh trục X, thì dùng dòng này:
    // float accel_pitch = atan2((float)acc_y, sqrt((float)acc_x * acc_x + (float)acc_z * acc_z)) * RAD_TO_DEG;

    // 3. TÍNH VẬN TỐC GÓC (GYRO RATE)
    // 131.0 là độ nhạy tương ứng với +/- 250 deg/s
    // Nhớ thay gyro_x bằng trục thực tế mà robot bạn đang xoay quanh
    float gyro_rate = gyro_x / 131.0f; 

    // 4. ÁP DỤNG COMPLEMENTARY FILTER
    current_pitch = ALPHA * (current_pitch + gyro_rate * dt) + (1.0f - ALPHA) * accel_pitch;

    return current_pitch;
}

float mpu6050_get_cached_pitch(void) {
    return current_pitch;
}
