/* 只用于电脑上的 GPIO 测试。固件编译使用 SysConfig 生成的真实头文件。 */
#ifndef TEST_MSP_CONFIG_H
#define TEST_MSP_CONFIG_H
#include <stdint.h>
#define LINE_PORT 0U
#define LINE_S0_PIN (1U << 0)
#define LINE_S1_PIN (1U << 1)
#define LINE_S2_PIN (1U << 4)
#define LINE_S3_PIN (1U << 5)
#define LINE_S4_PIN (1U << 8)
#define LINE_S5_PIN (1U << 9)
#define LINE_S6_PIN (1U << 10)
#define LINE_S7_PIN (1U << 11)
extern uint32_t test_gpio_input;
static inline uint32_t DL_GPIO_readPins(unsigned port, uint32_t pins)
{
    (void)port;
    return test_gpio_input & pins;
}
#endif
