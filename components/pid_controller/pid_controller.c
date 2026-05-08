#include "pid_controller.h"

void pid_init(pid_context_t *pid, float kp, float ki, float kd, float max_output) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error_sum = 0.0f;
    pid->last_error = 0.0f;
    pid->max_output = max_output;
}

void pid_reset(pid_context_t *pid) {
    pid->error_sum = 0.0f;
    pid->last_error = 0.0f;
}

float pid_compute(pid_context_t *pid, float setpoint, float measured_value, float dt) {
    // 1. Tính sai số hiện tại
    float error = setpoint - measured_value;

    // 2. Tính khâu Tỉ lệ (P)
    float p_out = pid->kp * error;

    // 3. Tính khâu Tích phân (I) với Anti-Windup (Chống tràn)
    pid->error_sum += error * dt;
    float i_out = pid->ki * pid->error_sum;
    
    // Giới hạn khâu I để không bị bão hòa khi robot ngã quá lâu
    if (i_out > pid->max_output) { i_out = pid->max_output; pid->error_sum = i_out / pid->ki; }
    else if (i_out < -pid->max_output) { i_out = -pid->max_output; pid->error_sum = i_out / pid->ki; }

    // 4. Tính khâu Vi phân (D)
    float derivative = (error - pid->last_error) / dt;
    float d_out = pid->kd * derivative;
    pid->last_error = error;

    // 5. Tổng hợp đầu ra
    float output = p_out + i_out + d_out;

    // 6. Giới hạn tốc độ động cơ tối đa
    if (output > pid->max_output) output = pid->max_output;
    else if (output < -pid->max_output) output = -pid->max_output;

    return output;
}