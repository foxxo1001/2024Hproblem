#include "ti_msp_dl_config.h"
#include "uart.h"

void UART_SendString(const char *str)
{
    while (*str != '\0') {
        DL_UART_transmitDataBlocking(PRINT_INST, (uint8_t)*str++);
    }
}
