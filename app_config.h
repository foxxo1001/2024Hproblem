#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 实测后填写，0 表示未标定，禁止启动路线。每圈计数按 A 相上升沿统计。 */
#define CAR_WHEEL_DIAMETER_MM           0.0f
#define CAR_LEFT_COUNTS_PER_WHEEL_REV   0.0f
#define CAR_RIGHT_COUNTS_PER_WHEEL_REV  0.0f
#define CAR_WHEEL_TRACK_MM              0.0f

/* 电机、编码器前进为正；陀螺仪左转为正。 */
#define MOTOR_LEFT_FORWARD_SIGN          1
#define MOTOR_RIGHT_FORWARD_SIGN        -1
#define ENCODER_LEFT_SIGN                1
#define ENCODER_RIGHT_SIGN              -1
#define IMU_YAW_SIGN                     1.0f
#define MPU6050_ADDRESS                  0x68U
#define LINE_BLACK_LEVEL                 0U
#define BUZZER_ACTIVE_LEVEL              1U

/* 速度 PID 每 10ms 运行；KI 已包含采样周期。参数仅作为起调值。 */
#define CONTROL_PERIOD_MS               10U
#define CONTROL_DT                      0.01f
#define MOTOR_PWM_LIMIT                 2800U
#define SPEED_PID_KP                    45.0f
#define SPEED_PID_KI                     6.0f
#define SPEED_PID_KD                     0.0f
#define SPEED_PID_IMAX                  2400.0f

/* 轮速单位 mm/s；航向误差单位 rad；灰度误差单位为探头间距。 */
#define CAR_STRAIGHT_SPEED_MM_S         260.0f
#define CAR_ARC_SPEED_MM_S              210.0f
#define CAR_TURN_SPEED_MM_S             110.0f
#define CAR_MAX_WHEEL_SPEED_MM_S        340.0f
#define CAR_HEADING_KP                  300.0f
#define CAR_YAW_DAMPING                  15.0f
#define CAR_LINE_KP                     35.0f
#define CAR_LINE_KD                     18.0f
#define CAR_SENSOR_FORWARD_MM           50.0f /* 灰度排距轮轴的距离 */
#define CAR_STOP_LEAD_MM                  0.0f /* 停车提前量，需实测 */
#endif
