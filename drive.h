#ifndef DRIVE_H
#define DRIVE_H
#include "car.h"
#include "pid.h"
typedef struct {
    pid_ctrl_t left_pid;          /* 左右轮各有一套 PID 积分等记录 */
    pid_ctrl_t right_pid;
    float left_ramped_mm_s;       /* 平滑加减速后的目标轮速 */
    float right_ramped_mm_s;
    uint16_t left_pwm;
    uint16_t right_pwm;
} drive_t;
void drive_init(drive_t *drive);
void drive_update(drive_t *drive, const car_t *car, const car_input_t *input); /* 仅行驶时调用 */
void drive_stop(drive_t *drive, bool brake);
#endif
