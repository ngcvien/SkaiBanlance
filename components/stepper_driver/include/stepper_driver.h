#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <stdint.h>

void stepper_driver_init(void);
void stepper_set_speed(float speed_left, float speed_right);

// Lấy vị trí hiện tại của bánh xe (Bộ đếm Encoder ảo)
int32_t stepper_get_left_step(void);
int32_t stepper_get_right_step(void);

// Reset bộ đếm khi robot ngã
void stepper_reset_step(void);

#endif // STEPPER_DRIVER_H