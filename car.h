#ifndef CAR_H
#define CAR_H
#include <stdbool.h>
#include <stdint.h>
#include "line_sensor.h"

typedef enum {
    CAR_IDLE,       /* 待机 */
    CAR_TURN,       /* 转到下一段方向 */
    CAR_STRAIGHT,   /* 陀螺仪保持直行 */
    CAR_ARC,        /* 灰度循迹走半圆 */
    CAR_DONE, CAR_FAULT
} car_state_t;

typedef enum {
    CAR_OK, CAR_ERR_CONFIG, CAR_ERR_IMU, CAR_ERR_LINE,
    CAR_ERR_STALL, CAR_ERR_DIRECTION, CAR_ERR_TIMEOUT, CAR_ERR_DISTANCE
} car_fault_t;


typedef struct {
    float left_mm_per_count, right_mm_per_count;
    float track_mm;
} car_geometry_t;

typedef struct {
    int32_t left_count, right_count; /* 这 10ms 的编码器计数 */
    float yaw_rate_rad_s;           /* 左转为正，单位 rad/s */
    bool imu_ok;
    line_sample_t line;
} car_input_t;

typedef struct {
    car_geometry_t geometry;
    car_state_t state;
    car_fault_t fault;
    uint8_t task, segment, laps_done; /* 项目、当前路段、完成圈数 */
    uint8_t point_events;            /* 到顶点的累计次数，用于提示 */
    char point;
    uint32_t elapsed_ms, segment_ms;
    float yaw_rad, heading_rad;     /* 实际车头角度、目标角度，单位 rad */
    float distance_mm;              /* 当前路段的双轮平均里程 */
    float left_target_mm_s, right_target_mm_s;
    float last_line_error;
    bool line_seen, line_previous_valid;
    uint16_t lost_line_ms, stalled_ms[2], reversed_ms[2];
} car_t;

void car_init(car_t *car, car_geometry_t geometry);
bool car_geometry_valid(const car_t *car);
bool car_running(const car_t *car);
bool car_start(car_t *car, uint8_t task, bool imu_ok);
void car_stop(car_t *car);
void car_fail(car_t *car, car_fault_t fault);
void car_tick(car_t *car, const car_input_t *input); /* 每 10ms 调用 */
const char *car_state_name(car_state_t state);
const char *car_fault_name(car_fault_t fault);
#endif
