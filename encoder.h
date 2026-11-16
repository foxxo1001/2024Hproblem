#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>
void encoder_init(void);
void encoder_read_pair(int32_t *left, int32_t *right);
void encoder_total_pair(int32_t *left, int32_t *right);
#endif
