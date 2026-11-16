#include "ti_msp_dl_config.h"
#include "key.h"

uint8_t getkeysta(uint32_t key)
{
    return DL_GPIO_readPins(KEY_PORT, key) != 0U;
}

uint8_t key_scan_pressed(void)
{
    static uint8_t stable[2], different[2];
    const uint32_t pins[2] = {KEY_KEY1_PIN, KEY_KEY2_PIN};
    uint8_t events = 0;
    for (uint8_t i = 0; i < 2; ++i) {
        uint8_t raw = getkeysta(pins[i]);
        if (raw == stable[i]) different[i] = 0;
        else if (++different[i] >= 3U) {
            different[i] = 0;
            stable[i] = raw;
            if (raw) events |= (uint8_t)(1U << i);
        }
    }
    return events;
}
