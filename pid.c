#include "pid.h"

void PID_clear(pid_ctrl_t *pid)
{
    pid->i_out = 0;
    pid->last_error = 0;
}

void PID_init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float max_out, float max_iout)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    PID_clear(pid);
}

float PID_calc(pid_ctrl_t *pid, float feedback, float target)
{
    float error = target - feedback;
    float p = pid->kp * error;
    float d = pid->kd * (error - pid->last_error);
    float next_i = pid->i_out + pid->ki * error;
    pid->last_error = error;
    if (next_i > pid->max_iout) next_i = pid->max_iout;
    if (next_i < -pid->max_iout) next_i = -pid->max_iout;

    /* PWM 顶到上限时不继续积正误差，降到 0 时不继续积负误差。 */
    float output = p + next_i + d;
    if (!(output > pid->max_out && error > 0) && !(output < 0 && error < 0))
        pid->i_out = next_i;
    output = p + pid->i_out + d;
    if (output > pid->max_out) output = pid->max_out;
    if (output < 0) output = 0;
    return output;
}
