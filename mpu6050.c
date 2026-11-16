#include "ti_msp_dl_config.h"
#include "mpu6050.h"
#include "app_config.h"
#include "delay.h"
#include <stdint.h>

/* 寄存器相当于 MPU6050 内部的小格子：按地址写配置、读测量值。 */
#define REG_SAMPLE_DIVIDER  0x19
#define REG_FILTER          0x1A
#define REG_GYRO_RANGE      0x1B
#define REG_GYRO_Z_HIGH     0x47
#define REG_POWER_1         0x6B
#define REG_POWER_2         0x6C
#define REG_WHO_AM_I        0x75
#define GYRO_LSB_PER_DPS    65.5f  /* 配置成 ±500°/s 后的换算系数 */
#define DEG_TO_RAD          0.01745329252f

static float gyro_z_bias; /* 静止时的 Z 轴平均读数，之后从测量值里减去 */

static bool wait_done(uint32_t interrupt)
{
    /* 等硬件把数据发完/收完，最多等约 1ms。
     * 这里保留超时：如果线松了，不能一直卡在中断里，电机会停不下来。
     */
    for (unsigned us = 0; us < 1000U; ++us) {
        uint32_t status = DL_I2C_getControllerStatus(IMU_INST);
        if ((status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                       DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)) != 0U) break;
        if (DL_I2C_getRawInterruptStatus(IMU_INST, interrupt) != 0U) {
            DL_I2C_clearInterruptStatus(IMU_INST, interrupt);
            return true;
        }
        delay_cycles(CPUCLK_FREQ / 1000000U);
    }
    DL_I2C_resetControllerTransfer(IMU_INST);
    return false;
}

static bool bus_idle(void)
{
    for (unsigned us = 0; us < 1000U; ++us) {
        if ((DL_I2C_getControllerStatus(IMU_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U)
            return true;
        delay_cycles(CPUCLK_FREQ / 1000000U);
    }
    return false;
}

static bool write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    if (!bus_idle()) return false;
    DL_I2C_flushControllerTXFIFO(IMU_INST);
    DL_I2C_clearInterruptStatus(IMU_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    DL_I2C_fillControllerTXFIFO(IMU_INST, data, 2);
    DL_I2C_startControllerTransfer(IMU_INST, MPU6050_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    if (!wait_done(DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) return false;
    return bus_idle();
}

static bool read_regs(uint8_t reg, uint8_t *data, unsigned length)
{
    /* 第一步：告诉 MPU6050，接下来要读哪个寄存器。 */
    if (!bus_idle()) return false;
    DL_I2C_flushControllerTXFIFO(IMU_INST);
    DL_I2C_flushControllerRXFIFO(IMU_INST);
    DL_I2C_clearInterruptStatus(IMU_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
                                          DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    DL_I2C_fillControllerTXFIFO(IMU_INST, &reg, 1);
    DL_I2C_startControllerTransferAdvanced(IMU_INST, MPU6050_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1, DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE, DL_I2C_CONTROLLER_ACK_DISABLE);
    if (!wait_done(DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) return false;
    /* 第二步：切换为读取，把指定寄存器开始的 length 个字节收回来。 */
    DL_I2C_startControllerTransferAdvanced(IMU_INST, MPU6050_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_RX, (uint16_t)length, DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_DISABLE);
    if (!wait_done(DL_I2C_INTERRUPT_CONTROLLER_RX_DONE)) return false;
    if (!bus_idle()) return false;
    for (unsigned i = 0; i < length; ++i) {
        if (DL_I2C_isControllerRXFIFOEmpty(IMU_INST)) return false;
        data[i] = DL_I2C_receiveControllerData(IMU_INST);
    }
    return true;
}

static bool raw_z(int16_t *value)
{
    uint8_t bytes[2];
    if (!read_regs(REG_GYRO_Z_HIGH, bytes, 2)) return false;
    /* 先收到高 8 位，再收到低 8 位，拼成一个带正负号的 16 位数。 */
    *value = (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
    return true;
}

bool mpu6050_init_and_calibrate(void)
{
    uint8_t identity;
    if (!read_regs(REG_WHO_AM_I, &identity, 1)) return false;
    if (identity != 0x68U) return false;
    if (!write_reg(REG_POWER_1, 0x80)) return false; /* 先复位芯片 */
    delay_ms(100);
    const uint8_t settings[][2] = {
        /* 寄存器地址          写入值   作用 */
        {REG_POWER_1,         0x01}, /* 解除睡眠，选择时钟 */
        {REG_POWER_2,         0x00}, /* 开启各轴 */
        {REG_FILTER,          0x03}, /* 低通滤波，约 42Hz */
        {REG_SAMPLE_DIVIDER,  0x09}, /* 每秒采样 100 次 */
        {REG_GYRO_RANGE,      0x08}  /* 量程 ±500°/s */
    };
    for (unsigned i = 0; i < 5; ++i) {
        if (!write_reg(settings[i][0], settings[i][1])) return false;
    }
    delay_ms(800);
    int32_t sum = 0;
    int16_t minimum = INT16_MAX, maximum = INT16_MIN;
    for (unsigned i = 0; i < 200; ++i) {
        int16_t value;
        if (!raw_z(&value)) return false;
        sum += value;
        if (value < minimum) minimum = value;
        if (value > maximum) maximum = value;
        delay_ms(10);
    }
    /* 静止读 200 次取平均，得到零偏；明显在转动时，本次校准不采用。 */
    gyro_z_bias = (float)sum / 200.0f;
    return ((int32_t)maximum - minimum) < 328
        && gyro_z_bias > -1310.0f && gyro_z_bias < 1310.0f;
}

bool mpu6050_read_yaw_rate(float *rad_s)
{
    int16_t value;
    if (!raw_z(&value)) return false;
    float degrees_per_second = ((float)value - gyro_z_bias) / GYRO_LSB_PER_DPS;
    *rad_s = degrees_per_second * DEG_TO_RAD * IMU_YAW_SIGN;
    return true;
}
