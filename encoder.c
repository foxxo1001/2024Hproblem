#include "ti_msp_dl_config.h"
#include "encoder.h"
#include "app_config.h"

/* 下标 0 表示左轮，1 表示右轮。
 * counts：最近 10ms 的计数，读完清零，用于测速。
 * totals：从上电开始的累计计数，手动转轮标定时看这个值。
 * volatile 表示这些变量随时可能被中断修改。
 */
static volatile int32_t counts[2];
static volatile int32_t totals[2];

void encoder_init(void)
{
    DL_GPIO_clearInterruptStatus(GPIOA, DC_MOTOR_AA_PIN | DC_MOTOR_BA_PIN);
    NVIC_ClearPendingIRQ(DC_MOTOR_INT_IRQN);
    NVIC_SetPriority(DC_MOTOR_INT_IRQN, 0);
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);
}

void encoder_read_pair(int32_t *left, int32_t *right)
{
    /* 复制、清零的几次赋值期间暂停中断，避免刚来的脉冲被清掉。 */
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    *left = counts[0];
    *right = counts[1];
    counts[0] = counts[1] = 0;
    __set_PRIMASK(saved);
}

void encoder_total_pair(int32_t *left, int32_t *right)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    *left = totals[0];
    *right = totals[1];
    __set_PRIMASK(saved);
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case DL_INTERRUPT_GROUP1_IIDX_GPIOA: {
            uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOA,
                DC_MOTOR_AA_PIN | DC_MOTOR_BA_PIN);
            DL_GPIO_clearInterruptStatus(GPIOA, status);
            /* 左轮 A 相来了一个上升沿：看 B 相电平，决定加 1 还是减 1。 */
            if ((status & DC_MOTOR_AA_PIN) != 0U) {
                int32_t step = DL_GPIO_readPins(DC_MOTOR_AB_PORT, DC_MOTOR_AB_PIN)
                    ? ENCODER_LEFT_SIGN : -ENCODER_LEFT_SIGN;
                counts[0] += step;
                totals[0] += step;
            }
            /* 右轮做同样的事情，方向由自己的 SIGN 宏修正。 */
            if ((status & DC_MOTOR_BA_PIN) != 0U) {
                int32_t step = DL_GPIO_readPins(DC_MOTOR_BB_PORT, DC_MOTOR_BB_PIN)
                    ? ENCODER_RIGHT_SIGN : -ENCODER_RIGHT_SIGN;
                counts[1] += step;
                totals[1] += step;
            }
            break;
        }
        default: break;
    }
}
