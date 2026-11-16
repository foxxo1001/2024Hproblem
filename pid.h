#ifndef PID_H
#define PID_H

/* 速度单位：计数/10ms；输出：0~max_out 的正向 PWM。 */
typedef struct {
    float kp, ki, kd;
    float max_out, max_iout;
    float i_out, last_error; /* 只保存下一次计算需要的两个值 */
} pid_ctrl_t;

void PID_init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float max_out, float max_iout);
void PID_clear(pid_ctrl_t *pid);
float PID_calc(pid_ctrl_t *pid, float feedback, float target);
#endif
