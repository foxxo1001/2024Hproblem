#ifndef MPU6050_H
#define MPU6050_H
#include <stdbool.h>
bool mpu6050_init_and_calibrate(void); /* 车保持静止，约 3 秒 */
bool mpu6050_read_yaw_rate(float *rad_s);
#endif
