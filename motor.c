#include "ti_msp_dl_config.h"
#include "motor.h"
#include "app_config.h"

static void pwm_write(uint16_t left, uint16_t right)
{
    if (left > MOTOR_PWM_LIMIT) left = MOTOR_PWM_LIMIT;
    if (right > MOTOR_PWM_LIMIT) right = MOTOR_PWM_LIMIT;
    DL_Timer_setCaptureCompareValue(PWMA_INST, left, GPIO_PWMA_C0_IDX);
    DL_Timer_setCaptureCompareValue(PWMA_INST, right, GPIO_PWMA_C1_IDX);
}

void motor_disable(void)
{
    DL_GPIO_clearPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
    pwm_write(0, 0);
}

void motor_init(void)
{
    motor_disable();
}

void motor_brake(void)
{
    /* TB6612 IN1=IN2=高：短路制动，不能用反转代替刹车。 */
    pwm_write(0, 0);
    DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN | DC_MOTOR_AIN2_PIN);
    DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN | DC_MOTOR_BIN2_PIN);
    DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
}

void motor_set_forward(uint16_t left, uint16_t right)
{
    /* 镜像安装的电机电气方向可能相反，宏应按实际前进方向标定。 */
#if MOTOR_LEFT_FORWARD_SIGN > 0
    DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
    DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
#else
    DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
    DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
#endif
#if MOTOR_RIGHT_FORWARD_SIGN > 0
    DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
    DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
#else
    DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
    DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
#endif
    pwm_write(left, right);
    DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
}
