#ifndef KEY_H
#define KEY_H
#include <stdint.h>
uint8_t getkeysta(uint32_t key);
uint8_t key_scan_pressed(void); /* 每 10ms 调用：bit0 KEY1、bit1 KEY2 单次按下 */
#endif
