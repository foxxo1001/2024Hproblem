#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 必填的实车参数。JGB37-520 有多种减速比，不能根据名字推算脉冲数。
 * 当前只数 A 相上升沿，不能填四倍频计数。
 * 0 = 未标定：可查看传感器，但拒绝启动比赛路线。
 */
#define CAR_WHEEL_DIAMETER_MM           0.0f
#define CAR_LEFT_COUNTS_PER_WHEEL_REV   0.0f
#define CAR_RIGHT_COUNTS_PER_WHEEL_REV  0.0f
#define CAR_WHEEL_TRACK_MM              0.0f

/* 正数必须对应车身前进。左右独立标定，不能只取绝对值掩盖反向。 */
#define MOTOR_LEFT_FORWARD_SIGN          1
#define MOTOR_RIGHT_FORWARD_SIGN        -1
#define ENCODER_LEFT_SIGN                1
#define ENCODER_RIGHT_SIGN              -1
#define IMU_YAW_SIGN                     1.0f /* 手动左转，角度应增加 */
#define MPU6050_ADDRESS                  0x68U /* AD0=1 改为 0x69 */
#define LINE_BLACK_LEVEL                 0U    /* 实测黑=高则改为 1 */
#define BUZZER_ACTIVE_LEVEL              1U    /* 有源蜂鸣器/驱动模块 */
#define CAR_USE_OLED                     1

#define CONTROL_PERIOD_MS               10U
#define CONTROL_DT                      0.01f
#define MOTOR_PWM_PERIOD                4000U
#define MOTOR_PWM_LIMIT                 2800U /* 调试限幅，不是电流限制 */
#define SPEED_PID_KP                    45.0f /* 反馈单位：计数/10 ms */
#define SPEED_PID_KI                     6.0f /* 离散积分，含采样周期 */
#define SPEED_PID_KD                     0.0f
#define SPEED_PID_IMAX                  2400.0f
#define SPEED_ACCEL_MM_S2               450.0f
#define CAR_STRAIGHT_SPEED_MM_S         260.0f
#define CAR_ARC_SPEED_MM_S              210.0f
#define CAR_TURN_SPEED_MM_S             110.0f
#define CAR_MAX_WHEEL_SPEED_MM_S        340.0f
#define CAR_HEADING_KP                  300.0f /* mm/s/rad，输出半轮速差 */
#define CAR_CROSSTRACK_KP                 1.5f /* mm/s/mm */
#define CAR_YAW_DAMPING                  15.0f /* mm/s/(rad/s) */
#define CAR_LINE_KP                     35.0f /* mm/s/灰度位置格 */
#define CAR_LINE_KD                     18.0f /* 每 10ms 的位置差分 */
#define CAR_SENSOR_FORWARD_MM           50.0f /* 灰度排距轮轴，需实测 */
#define CAR_STOP_LEAD_MM                  0.0f /* 刹车实测超出多少填多少 */

#endif
