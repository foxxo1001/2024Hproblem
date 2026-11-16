#include "drive.h"
#include "motor.h"
#include "app_config.h"
#include <string.h>

void drive_init(drive_t *drive)
{
    memset(drive, 0, sizeof(*drive));
    PID_init(&drive->left_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
        MOTOR_PWM_LIMIT, SPEED_PID_IMAX);
    PID_init(&drive->right_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
        MOTOR_PWM_LIMIT, SPEED_PID_IMAX);
}

/* 每次只改变一点车速，避免从静止一下跳到全速。
 * 例如加速度 450mm/s²，10ms 就允许改变 4.5mm/s。
 */
static float change_speed_slowly(float current, float target)
{
    float step = SPEED_ACCEL_MM_S2 * CONTROL_DT;
    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}

void drive_stop(drive_t *drive, bool brake)
{
    drive->left_ramped_mm_s = 0;
    drive->right_ramped_mm_s = 0;
    drive->left_pwm = 0;
    drive->right_pwm = 0;
    PID_clear(&drive->left_pid);
    PID_clear(&drive->right_pid);

    if (brake) motor_brake();
    else motor_disable();
}

/* 主程序只在行驶时调用，目标轮速由 car_tick 算出，且两轮均向前。
 * 停车直接用 drive_stop，不把 0 速度送进这个换算流程。
 */
void drive_update(drive_t *drive, const car_t *car, const car_input_t *input)
{
    /* 1. 先平滑车身速度，保持左右轮速比例不变，转弯半径才不会乱。 */
    float target_speed = (car->left_target_mm_s + car->right_target_mm_s) / 2;
    float current_speed = (drive->left_ramped_mm_s + drive->right_ramped_mm_s) / 2;
    float new_speed = change_speed_slowly(current_speed, target_speed);
    float scale = new_speed / target_speed;

    float faster_wheel = car->left_target_mm_s;
    if (car->right_target_mm_s > faster_wheel) faster_wheel = car->right_target_mm_s;
    if (faster_wheel * scale > CAR_MAX_WHEEL_SPEED_MM_S) {
        scale = CAR_MAX_WHEEL_SPEED_MM_S / faster_wheel;
    }
    drive->left_ramped_mm_s = car->left_target_mm_s * scale;
    drive->right_ramped_mm_s = car->right_target_mm_s * scale;

    /* 2. mm/s 换成编码器计数/10ms，才能和反馈比较。
     * 例如 200mm/s、每计数0.5mm：目标 = 200*0.01/0.5 = 4。
     */
    float left_target_count = drive->left_ramped_mm_s * CONTROL_DT
                            / car->geometry.left_mm_per_count;
    float right_target_count = drive->right_ramped_mm_s * CONTROL_DT
                             / car->geometry.right_mm_per_count;

    /* 3. 左右各算一次 PID，两个电机各管自己的速度。 */
    drive->left_pwm = (uint16_t)PID_calc(&drive->left_pid,
        (float)input->left_count, left_target_count);
    drive->right_pwm = (uint16_t)PID_calc(&drive->right_pid,
        (float)input->right_count, right_target_count);

    /* 4. 把本次算出的 PWM 写到 TB6612。 */
    motor_set_forward(drive->left_pwm, drive->right_pwm);
}
