#ifndef CAR_H
#define CAR_H

#include <stdbool.h>
#include <stdint.h>

/* 小车现在在做什么。前三个行驶状态分别交给三个控制函数。 */
typedef enum {
    CAR_IDLE,       /* 等待按键 */
    CAR_TURN,       /* 在顶点附近，向下一段的方向转弯 */
    CAR_STRAIGHT,   /* 沿没有黑线的直线/斜线前进 */
    CAR_ARC,        /* 用灰度传感器跟随黑色半圆 */
    CAR_DONE,       /* 任务完成 */
    CAR_FAULT       /* 出现故障，停车 */
} car_state_t;

typedef enum {
    CAR_OK, CAR_ERR_CONFIG, CAR_ERR_IMU, CAR_ERR_LINE,
    CAR_ERR_STALL, CAR_ERR_DIRECTION, CAR_ERR_TIMEOUT, CAR_ERR_DISTANCE
} car_fault_t;

typedef struct {
    uint8_t mask;       /* 8 个探头的结果，bit0 是最左边，1 表示黑色 */
    uint8_t count;      /* 当前有几个探头在黑线上 */
    bool valid;         /* 能否用这一帧计算黑线位置 */
    float error;        /* 黑线偏左为正，偏右为负，居中为 0 */
} line_sample_t;

typedef struct {
    float left_mm_per_count;   /* 左编码器每增加 1，小车左轮走多少 mm */
    float right_mm_per_count;  /* 右编码器每增加 1，小车右轮走多少 mm */
    float track_mm;            /* 左右轮接地中心的距离 */
} car_geometry_t;

/* 每隔 10ms 从硬件读取一次，交给 car_tick。 */
typedef struct {
    int32_t left_count;     /* 这 10ms 内左轮走过的编码器计数 */
    int32_t right_count;
    float yaw_rate_rad_s;  /* MPU6050 测得的转动速度，左转为正，单位 rad/s */
    bool imu_ok;
    line_sample_t line;
} car_input_t;

/* 小车的运行记录。通常只需要关注 state、segment 和两个目标轮速。 */
typedef struct {
    car_geometry_t geometry;
    car_state_t state;
    car_fault_t fault;
    uint8_t task;           /* 按键选择的项目，1~4 */
    uint8_t segment;        /* 当前路线表的第几行，0~3 */
    uint8_t laps_done;
    uint8_t point_events;   /* 每到一个顶点加 1，主程序据此响一次蜂鸣器 */
    char point;            /* 最近经过的 A/B/C/D */
    uint32_t elapsed_ms;   /* 本次任务已用时间 */
    uint32_t segment_ms;   /* 当前路段已用时间 */

    float yaw_rad;          /* 相对出发方向，已经转过多少角度，单位 rad */
    float heading_rad;      /* 当前路段希望车头朝向的角度 */
    float arc_end_yaw_rad;  /* 走完半圆时，车头应达到的角度 */
    float distance_mm;      /* 当前路段累计走过的距离，含接入时的转弯 */
    float projected_mm;     /* 沿目标直线方向前进了多少 mm */
    float crosstrack_mm;    /* 偏离目标直线多少 mm，左侧为正 */
    float left_target_mm_s;
    float right_target_mm_s;

    /* 循迹 PD 所需的上一次数据。 */
    float last_line_error;
    bool line_seen;
    bool line_previous_valid;

    /* 停车保护的计时，数组 [0] 是左轮，[1] 是右轮。 */
    uint16_t lost_line_ms;
    uint16_t stalled_ms[2];
    uint16_t reversed_ms[2];
} car_t;

line_sample_t line_decode(uint8_t black_mask);
void car_init(car_t *car, car_geometry_t geometry);
bool car_geometry_valid(const car_t *car);
bool car_running(const car_t *car);
bool car_start(car_t *car, uint8_t task, bool imu_ok); /* task 由按键限制为 1~4 */
void car_stop(car_t *car);
void car_fail(car_t *car, car_fault_t fault);
void car_tick(car_t *car, const car_input_t *input); /* 每 10ms 调用一次 */
const char *car_state_name(car_state_t state);
const char *car_fault_name(car_fault_t fault);

#endif
