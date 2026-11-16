#include "car.h"
#include "app_config.h"
#include <math.h>
#include <string.h>

#define PI_F             3.14159265359f
#define DIAGONAL_ANGLE   0.6747409422f  /* 38.66 度 */
#define DIAGONAL_LENGTH  1280.6248475f
#define ARC_RADIUS_MM    400.0f

/* turn=0 是直线，+1 是左半圆，-1 是右半圆。方向以出发车头为 0。 */
typedef struct {
    float heading;
    float length;
    int turn;
    char end_point;
} road_t;

static const road_t routes[2][4] = {
    { /* M1 只走第一段；M2 走完整圈 A-B-C-D-A。 */
        {0,      1000, 0, 'B'},
        {0,      PI_F * ARC_RADIUS_MM, -1, 'C'},
        {-PI_F,  1000, 0, 'D'},
        {-PI_F,  PI_F * ARC_RADIUS_MM, -1, 'A'}
    },
    { /* M3 走一圈 A-C-B-D-A；M4 走四圈。 */
        {0,                        DIAGONAL_LENGTH, 0, 'C'},
        {DIAGONAL_ANGLE,            PI_F * ARC_RADIUS_MM, 1, 'B'},
        {PI_F + 2 * DIAGONAL_ANGLE, DIAGONAL_LENGTH, 0, 'D'},
        {PI_F + DIAGONAL_ANGLE,     PI_F * ARC_RADIUS_MM, -1, 'A'}
    }
};

static float angle_error(float angle)
{
    while (angle > PI_F) angle -= 2 * PI_F;
    while (angle < -PI_F) angle += 2 * PI_F;
    return angle;
}

line_sample_t line_decode(uint8_t mask)
{
    line_sample_t line = {0};
    float sum = 0;
    unsigned count = 0, groups = 0;
    bool previous_black = false;
    line.mask = mask;
    for (unsigned i = 0; i < 8; ++i) {
        bool black = (mask & (1U << i)) != 0;
        if (black) {
            ++count;
            sum += 3.5f - i; /* 左边为正，右边为负，中间两路平均为 0。 */
            if (!previous_black) ++groups;
        }
        previous_black = black;
    }
    line.valid = count >= 1 && count <= 5 && groups == 1;
    if (line.valid) line.error = sum / count;
    return line;
}

bool car_geometry_valid(const car_t *car)
{
    return car->geometry.left_mm_per_count > 0
        && car->geometry.right_mm_per_count > 0 && car->geometry.track_mm > 0;
}

void car_init(car_t *car, car_geometry_t geometry)
{
    memset(car, 0, sizeof(*car));
    car->geometry = geometry;
    car->task = 1;
    car->point = 'A';
}

bool car_running(const car_t *car)
{
    return car->state == CAR_TURN || car->state == CAR_STRAIGHT || car->state == CAR_ARC;
}

void car_stop(car_t *car)
{
    car->state = CAR_IDLE;
    car->fault = CAR_OK;
    car->left_target_mm_s = car->right_target_mm_s = 0;
}

void car_fail(car_t *car, car_fault_t fault)
{
    car_stop(car);
    car->state = CAR_FAULT;
    car->fault = fault;
}

static void begin_segment(car_t *car)
{
    const road_t *road = &routes[car->task <= 2 ? 0 : 1][car->segment];
    float error = angle_error(road->heading - car->yaw_rad);
    car->heading_rad = car->yaw_rad + error;
    car->distance_mm = 0;
    car->segment_ms = car->lost_line_ms = 0;
    car->line_seen = car->line_previous_valid = false;
    car->last_line_error = 0;
    if (fabsf(error) > 0.025f) car->state = CAR_TURN;
    else car->state = road->turn == 0 ? CAR_STRAIGHT : CAR_ARC;
}

bool car_start(car_t *car, uint8_t task, bool imu_ok)
{
    if (car_running(car) || task < 1 || task > 4) return false;
    car_geometry_t geometry = car->geometry;
    car_init(car, geometry);
    car->task = task;
    if (!car_geometry_valid(car) || !imu_ok) {
        car_fail(car, !imu_ok ? CAR_ERR_IMU : CAR_ERR_CONFIG);
        return false;
    }
    car->point_events = 1;
    begin_segment(car);
    return true;
}

static void reach_point(car_t *car, const road_t *road)
{
    car->point = road->end_point;
    ++car->point_events;
    if (++car->segment == 4) {
        car->segment = 0;
        ++car->laps_done;
    }
    unsigned laps = car->task == 4 ? 4 : 1;
    if (car->task == 1 || car->laps_done == laps) {
        car_stop(car);
        car->state = CAR_DONE;
    } else {
        begin_segment(car);
    }
}

/* 左转修正为正：左轮慢一点，右轮快一点。两轮均不反转。 */
static void set_speed(car_t *car, float speed, float correction)
{
    float limit = speed * 0.92f;
    if (correction > limit) correction = limit;
    if (correction < -limit) correction = -limit;
    float left = speed - correction, right = speed + correction;
    float faster = fmaxf(left, right);
    if (faster > CAR_MAX_WHEEL_SPEED_MM_S) {
        left *= CAR_MAX_WHEEL_SPEED_MM_S / faster;
        right *= CAR_MAX_WHEEL_SPEED_MM_S / faster;
    }
    car->left_target_mm_s = left;
    car->right_target_mm_s = right;
}

static void check_safety(car_t *car, const car_input_t *input)
{
    const uint32_t timeout_ms[4] = {30000, 60000, 80000, 240000};
    if (car->elapsed_ms >= timeout_ms[car->task - 1]
        || (car->state == CAR_TURN && car->segment_ms > 3000)) {
        car_fail(car, CAR_ERR_TIMEOUT);
        return;
    }
    const int32_t counts[2] = {input->left_count, input->right_count};
    const float targets[2] = {car->left_target_mm_s, car->right_target_mm_s};
    for (unsigned i = 0; i < 2; ++i) {
        if (counts[i] < 0) car->reversed_ms[i] += CONTROL_PERIOD_MS;
        if (counts[i] > 0) car->reversed_ms[i] = 0;
        if (targets[i] > 60 && counts[i] == 0) car->stalled_ms[i] += CONTROL_PERIOD_MS;
        else car->stalled_ms[i] = 0;
        if (car->reversed_ms[i] >= 60) car_fail(car, CAR_ERR_DIRECTION);
        if (car->stalled_ms[i] >= 1200) car_fail(car, CAR_ERR_STALL);
        if (!car_running(car)) return;
    }
}

/* 每 10ms：更新角度和里程，然后只处理当前的一种动作。 */
void car_tick(car_t *car, const car_input_t *input)
{
    if (!input->imu_ok) {
        if (car_running(car)) car_fail(car, CAR_ERR_IMU);
        return;
    }
    car->yaw_rad += input->yaw_rate_rad_s * CONTROL_DT;
    if (!car_running(car)) return;
    car->distance_mm += (input->left_count * car->geometry.left_mm_per_count
                       + input->right_count * car->geometry.right_mm_per_count) / 2;
    car->elapsed_ms += CONTROL_PERIOD_MS;
    car->segment_ms += CONTROL_PERIOD_MS;
    check_safety(car, input);
    if (!car_running(car)) return;

    const road_t *road = &routes[car->task <= 2 ? 0 : 1][car->segment];
    float heading_error = angle_error(car->heading_rad - car->yaw_rad);
    bool final = car->task == 1 || (car->segment == 3
                && (car->task != 4 || car->laps_done == 3));
    if (car->distance_mm > road->length * 1.45f) {
        car_fail(car, CAR_ERR_DISTANCE);
        return;
    }
    if (car->state == CAR_TURN && fabsf(heading_error) <= 0.025f) {
        car->state = road->turn == 0 ? CAR_STRAIGHT : CAR_ARC;
    }

    switch (car->state) {
    case CAR_TURN: {
        /* 前进转向：依靠陀螺仪转到下一段方向，不原地反转。 */
        float correction = CAR_TURN_SPEED_MM_S / 1.1f;
        if (heading_error < 0) correction = -correction;
        set_speed(car, CAR_TURN_SPEED_MM_S, correction);
        break;
    }
    case CAR_STRAIGHT: {
        float remaining = road->length - car->distance_mm;
        if (final) remaining -= CAR_STOP_LEAD_MM;
        if (remaining <= 0) { reach_point(car, road); return; }
        float speed = CAR_STRAIGHT_SPEED_MM_S;
        if (remaining < 120) speed = 80 + (speed - 80) * remaining / 120;
        float correction = CAR_HEADING_KP * heading_error
                         - CAR_YAW_DAMPING * input->yaw_rate_rad_s;
        set_speed(car, speed, correction);
        break;
    }
    case CAR_ARC: {
        float remaining = PI_F - road->turn * (car->yaw_rad - car->heading_rad);
        bool travelled_enough = car->distance_mm > road->length * 0.70f;
        bool near_end = travelled_enough
                     && remaining < CAR_SENSOR_FORWARD_MM / ARC_RADIUS_MM + 0.10f;
        if (input->line.valid) {
            car->line_seen = true;
            car->lost_line_ms = 0;
        } else {
            car->lost_line_ms += CONTROL_PERIOD_MS;
            unsigned allowed = car->line_seen ? 150 : 800;
            if (car->lost_line_ms >= allowed && !(car->line_seen && near_end)) {
                car_fail(car, CAR_ERR_LINE);
                return;
            }
        }
        float stop_angle = final ? CAR_STOP_LEAD_MM / ARC_RADIUS_MM : 0;
        if (remaining <= 0.015f + stop_angle && travelled_enough && car->line_seen) {
            reach_point(car, road);
            return;
        }
        /* 按半径给基础轮速差，灰度 PD 把黑线拉回传感器中间。 */
        float speed = remaining < 0.25f ? 100 : CAR_ARC_SPEED_MM_S;
        float correction = road->turn * speed * car->geometry.track_mm / (2 * ARC_RADIUS_MM);
        if (input->line.valid) {
            float change = car->line_previous_valid ? input->line.error - car->last_line_error : 0;
            correction += CAR_LINE_KP * input->line.error + CAR_LINE_KD * change;
            car->last_line_error = input->line.error;
        }
        car->line_previous_valid = input->line.valid;
        set_speed(car, speed, correction);
        break;
    }
    default: break;
    }
}

const char *car_state_name(car_state_t state)
{
    static const char *const names[] = {"IDLE", "TURN", "STRAIGHT", "ARC", "DONE", "FAULT"};
    return names[state];
}

const char *car_fault_name(car_fault_t fault)
{
    static const char *const names[] = {"OK", "CFG", "IMU", "LINE", "STALL", "DIR", "TIME", "DIST"};
    return names[fault];
}
