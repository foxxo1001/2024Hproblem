#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>
void motor_init(void);
void motor_set_forward(uint16_t left, uint16_t right);
void motor_brake(void);
void motor_disable(void);
#endif
