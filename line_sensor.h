#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t mask;       /* bit0 在最左边，1=黑 */
    bool valid;
    float error;        /* 黑线偏左为正，偏右为负 */
} line_sample_t;

line_sample_t line_decode(uint8_t mask);
line_sample_t line_sensor_read(void);
#endif
