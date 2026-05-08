#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
    float kp;           // Hệ số Tỉ lệ (Sức mạnh phản ứng tức thời)
    float ki;           // Hệ số Tích phân (Khắc phục sai số cộng dồn)
    float kd;           // Hệ số Vi phân (Dự đoán và giảm chấn, chống rung)
    float error_sum;    // Tổng sai số (cho Ki)
    float last_error;   // Sai số lần trước (cho Kd)
    float max_output;   // Giới hạn tốc độ tối đa xuất ra động cơ
} pid_context_t;

// Khởi tạo thông số PID
void pid_init(pid_context_t *pid, float kp, float ki, float kd, float max_output);

// Tính toán đầu ra (Tốc độ động cơ) dựa trên góc hiện tại
float pid_compute(pid_context_t *pid, float setpoint, float measured_value, float dt);

// Reset bộ nháp tính toán (Dùng khi robot bị ngã và dựng lại)
void pid_reset(pid_context_t *pid);

#endif