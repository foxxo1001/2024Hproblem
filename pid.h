#ifndef PID_H
#define PID_H

/* 本项目的电机速度 PID：每 10ms 算一次，输出范围 0~max_out。
 * 速度目标和反馈都用“计数/10ms”，不直接用 rpm 或 mm/s。
 */
typedef struct {
    float kp;
    float ki;          /* 已包含 10ms 采样周期的离散积分系数 */
    float kd;
    float max_out;
    float max_iout;

    float set;         /* 目标速度 */
    float fdb;         /* 测得的速度 */
    float error;       /* 本次误差 */
    float last_error;  /* 上次误差，用于微分 */
    float p_out;
    float i_out;
    float d_out;
    float out;         /* 最终输出的 PWM */
} pid_ctrl_t;

void PID_init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float max_out, float max_iout);
void PID_clear(pid_ctrl_t *pid);
float PID_calc(pid_ctrl_t *pid, float feedback, float target);

#endif
