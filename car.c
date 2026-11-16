/* 先读 car_tick()；直行看 run_straight()，灰度循迹看 run_arc()。 */
#include "car.h"
#include "app_config.h"
#include <math.h>
#include <string.h>

#define PI_F             3.14159265359f
#define DIAGONAL_ANGLE   0.6747409422f  /* 38.66 度 */
#define DIAGONAL_LENGTH  1280.6248475f
#define ARC_RADIUS_MM    400.0f
#define ARC_LENGTH_MM    (PI_F * ARC_RADIUS_MM)

/* 四段固定为：直线、半圆、直线、半圆。这里只记录各段车头方向。 */
static const float normal_heading[4] = {0, 0, -PI_F, -PI_F};
static const float cross_heading[4] = {
    0, DIAGONAL_ANGLE, PI_F + 2 * DIAGONAL_ANGLE, PI_F + DIAGONAL_ANGLE
};

static float angle_error(float angle)
{
    while (angle > PI_F) angle -= 2 * PI_F;
    while (angle < -PI_F) angle += 2 * PI_F;
    return angle;
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

/* 进入新路段：里程清零，设置本段的车头方向。 */
static void begin_segment(car_t *car)
{
    float heading = normal_heading[car->segment];
    if (car->task >= 3) heading = cross_heading[car->segment];
    float error = angle_error(heading - car->yaw_rad);
    car->heading_rad = car->yaw_rad + error;
    car->distance_mm = 0;
    car->segment_ms = car->lost_line_ms = 0;
    car->line_seen = car->line_previous_valid = false;
    car->last_line_error = 0;
    car->state = car->segment % 2 == 0 ? CAR_STRAIGHT : CAR_ARC;
    if (fabsf(error) > 0.025f) car->state = CAR_TURN;
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

/* 到达 B/C/D/A 后换下一段，规定圈数走完就停车。 */
static void reach_point(car_t *car)
{
    const char *points = car->task <= 2 ? "ABCDA" : "ACBDA";
    car->point = points[car->segment + 1];
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

/* 直线：里程到终点就返回 true；陀螺仪负责纠偏。 */
static bool run_straight(car_t *car, const car_input_t *input, float length, float stop_lead)
{
    float remaining = length - car->distance_mm - stop_lead;
    if (remaining <= 0) return true;
    float speed = CAR_STRAIGHT_SPEED_MM_S;
    if (remaining < 120) speed = 80 + (speed - 80) * remaining / 120;
    float error = angle_error(car->heading_rad - car->yaw_rad);
    float correction = CAR_HEADING_KP * error - CAR_YAW_DAMPING * input->yaw_rate_rad_s;
    set_speed(car, speed, correction);
    return false;
}

/* 半圆：灰度纠偏，陀螺仪判断是否转过半圈。到终点返回 true。 */
static bool run_arc(car_t *car, const car_input_t *input, float stop_lead)
{
    int direction = -1; /* 普通路线顺时针；交叉路线 C->B 这一段逆时针。 */
    if (car->task >= 3 && car->segment == 1) direction = 1;
    float remaining = PI_F - direction * (car->yaw_rad - car->heading_rad);
    bool travelled_enough = car->distance_mm > ARC_LENGTH_MM * 0.70f;
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
            return false;
        }
    }
    if (remaining <= 0.015f + stop_lead / ARC_RADIUS_MM && travelled_enough && car->line_seen)
        return true;

    float speed = remaining < 0.25f ? 100 : CAR_ARC_SPEED_MM_S;
    float correction = direction * speed * car->geometry.track_mm / (2 * ARC_RADIUS_MM);
    if (input->line.valid) {
        float change = car->line_previous_valid ? input->line.error - car->last_line_error : 0;
        correction += CAR_LINE_KP * input->line.error + CAR_LINE_KD * change;
        car->last_line_error = input->line.error;
    }
    car->line_previous_valid = input->line.valid;
    set_speed(car, speed, correction);
    return false;
}

/* 每 10ms 只做三步：更新数据、检查停车条件、执行当前动作。 */
void car_tick(car_t *car, const car_input_t *input)
{
    /* 1. 更新陀螺仪角度和编码器里程。 */
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

    /* 2. 超时、堵转、反向计数或里程异常时停车。 */
    check_safety(car, input);
    if (!car_running(car)) return;
    float length = ARC_LENGTH_MM;
    if (car->segment % 2 == 0) length = car->task <= 2 ? 1000 : DIAGONAL_LENGTH;
    if (car->distance_mm > length * 1.45f) {
        car_fail(car, CAR_ERR_DISTANCE);
        return;
    }

    /* 3. 先转到目标方向，再直行或循迹。 */
    if (car->state == CAR_TURN) {
        float error = angle_error(car->heading_rad - car->yaw_rad);
        if (fabsf(error) > 0.025f) {
            float correction = CAR_TURN_SPEED_MM_S / 1.1f;
            if (error < 0) correction = -correction;
            set_speed(car, CAR_TURN_SPEED_MM_S, correction);
            return;
        }
        car->state = car->segment % 2 == 0 ? CAR_STRAIGHT : CAR_ARC;
    }
    unsigned laps = car->task == 4 ? 4 : 1;
    bool final = car->task == 1 || (car->segment == 3 && car->laps_done + 1U == laps);
    float stop_lead = final ? CAR_STOP_LEAD_MM : 0;
    bool arrived;
    if (car->state == CAR_STRAIGHT) arrived = run_straight(car, input, length, stop_lead);
    else arrived = run_arc(car, input, stop_lead);
    if (arrived) reach_point(car);
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
