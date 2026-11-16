#include "ti_msp_dl_config.h"
#include "line_sensor.h"
#include "app_config.h"

line_sample_t line_sensor_read(void)
{
    static const uint32_t pins[8] = {
        LINE_S0_PIN, LINE_S1_PIN, LINE_S2_PIN, LINE_S3_PIN,
        LINE_S4_PIN, LINE_S5_PIN, LINE_S6_PIN, LINE_S7_PIN
    };
    uint32_t levels = DL_GPIO_readPins(LINE_PORT, 0xFFFFFFFFU);
    uint8_t black = 0;
    for (unsigned i = 0; i < 8; ++i) {
        unsigned high = (levels & pins[i]) != 0U;
        /* 原始 DO 的高低电平，统一转换成“这一位为 1 就是黑色”。 */
        if (high == LINE_BLACK_LEVEL) black |= (uint8_t)(1U << i);
    }
    /* 例如中间两路为黑色，black=0x18，解算出来的位置误差是 0。 */
    return line_decode(black);
}
