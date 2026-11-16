#include "pid.h"

void PID_clear(pid_ctrl_t *pid)
{
    pid->set = 0;
    pid->fdb = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->p_out = 0;
    pid->i_out = 0;
    pid->d_out = 0;
    pid->out = 0;
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
    pid->set = target;
    pid->fdb = feedback;
    pid->error = target - feedback;

    /* 1. 比例 P：现在差多少速度，就补多少 PWM。 */
    pid->p_out = pid->kp * pid->error;

    /* 2. 积分 I：把过去没补足的误差逐次加起来。 */
    float next_i = pid->i_out + pid->ki * pid->error;
    if (next_i > pid->max_iout) next_i = pid->max_iout;
    if (next_i < -pid->max_iout) next_i = -pid->max_iout;

    /* 3. 微分 D：看误差的变化趋势，抑制来回摆动。 */
    pid->d_out = pid->kd * (pid->error - pid->last_error);
    pid->last_error = pid->error;

    /* PWM 已经顶到上限时，不继续累加正误差；降到 0 时同理。
     * 否则积分越积越多，目标降下来后电机还会继续加速。
     */
    float next_output = pid->p_out + next_i + pid->d_out;
    int pushing_upper_limit = next_output > pid->max_out && pid->error > 0;
    int pushing_lower_limit = next_output < 0 && pid->error < 0;
    if (!pushing_upper_limit && !pushing_lower_limit) {
        pid->i_out = next_i;
    }

    /* 4. P + I + D 得到 PWM；只能减到 0，不能输出反转。 */
    pid->out = pid->p_out + pid->i_out + pid->d_out;
    if (pid->out > pid->max_out) pid->out = pid->max_out;
    if (pid->out < 0) pid->out = 0;
    return pid->out;
}
