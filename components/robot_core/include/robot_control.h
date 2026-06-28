#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

typedef struct {
    float target_angle_deg;
    float turn_speed_steps_s;
} robot_motion_setpoint_t;

void robot_control_task_start(void);
void robot_control_get_setpoint(robot_motion_setpoint_t *setpoint);
void robot_control_emergency_stop(void);

#endif // ROBOT_CONTROL_H
