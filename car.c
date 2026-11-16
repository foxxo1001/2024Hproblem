#include "car.h"
#include "app_config.h"
#include <math.h>
#include <string.h>

#define PI_F              3.14159265359f
#define DIAGONAL_ANGLE    0.6747409422f   /* 38.66 度 = atan(800/1000) */
#define DIAGONAL_LENGTH   1280.6248475f   /* sqrt(1000*1000 + 800*800) */
#define ARC_RADIUS_MM     400.0f
#define ARC_LENGTH_MM     (PI_F * ARC_RADIUS_MM)
#define TURN_TOLERANCE    0.025f         /* 车头差约 1.4 度以内即可接下一段 */

/* 每一行是一段路：类型、起始方向、长度、弧线转向、终点。
 * 方向相对于出发时的车头；弧线转向 +1 为左转，-1 为右转。
 */
typedef enum { ROAD_STRAIGHT, ROAD_ARC } road_type_t;
typedef struct {
    road_type_t type;
    float heading;
    float length;
    int turn;
    char end_point;
} road_t;

/* 项目 1 只走第一行；项目 2 走完整张表。 */
static const road_t normal_route[4] = {
    /* 类型          方向        长度          转向  终点 */
    {ROAD_STRAIGHT,  0.0f,       1000.0f,         0, 'B'}, /* A->B */
    {ROAD_ARC,       0.0f,       ARC_LENGTH_MM,  -1, 'C'}, /* B->C */
    {ROAD_STRAIGHT, -PI_F,       1000.0f,         0, 'D'}, /* C->D */
    {ROAD_ARC,      -PI_F,       ARC_LENGTH_MM,  -1, 'A'}  /* D->A */
};

/* 项目 3、4：出发时车头对准 C，所以 A->C 的方向记作 0。 */
static const road_t cross_route[4] = {
    {ROAD_STRAIGHT, 0.0f,                         DIAGONAL_LENGTH, 0, 'C'},
    {ROAD_ARC,      DIAGONAL_ANGLE,               ARC_LENGTH_MM,   1, 'B'},
    {ROAD_STRAIGHT, PI_F + 2.0f * DIAGONAL_ANGLE, DIAGONAL_LENGTH, 0, 'D'},
    {ROAD_ARC,      PI_F + DIAGONAL_ANGLE,        ARC_LENGTH_MM,  -1, 'A'}
};

static const road_t *current_road(const car_t *car)
{
    if (car->task <= 2) {
        return &normal_route[car->segment];
    }
    return &cross_route[car->segment];
}

static unsigned target_laps(const car_t *car)
{
    if (car->task == 4) {
        return 4;
    }
    return 1;
}

/* 例如“向左转 350 度”和“向右转 10 度”等效，选后者。
 * 把角度差折回 -PI~PI，就能选到较短的转向。
 */
static float angle_error(float angle)
{
    while (angle > PI_F) angle -= 2.0f * PI_F;
    while (angle < -PI_F) angle += 2.0f * PI_F;
    return angle;
}

line_sample_t line_decode(uint8_t mask)
{
    const float position[8] = {3.5f, 2.5f, 1.5f, 0.5f, -0.5f, -1.5f, -2.5f, -3.5f};
    line_sample_t line = {0};
    float sum = 0;
    unsigned black_groups = 0;
    bool previous_black = false;
    line.mask = mask;

    for (unsigned i = 0; i < 8; ++i) {
        bool black = (mask & (1U << i)) != 0;
        if (black) {
            ++line.count;
            sum += position[i];
            if (!previous_black) ++black_groups;
        }
        previous_black = black;
    }

    /* 一条黑线应是一组连续黑点。全白、大片全黑和分散黑点不用来转向。 */
    line.valid = line.count >= 1 && line.count <= 5 && black_groups == 1;
    if (line.valid) line.error = sum / line.count;
    return line;
}

bool car_geometry_valid(const car_t *car)
{
    /* 启动前只检查必填参数是否为正，避免用未填写的 0 换算距离。 */
    return car->geometry.left_mm_per_count > 0
        && car->geometry.right_mm_per_count > 0
        && car->geometry.track_mm > 0;
}

void car_init(car_t *car, car_geometry_t geometry)
{
    memset(car, 0, sizeof(*car));
    car->geometry = geometry;
    car->state = CAR_IDLE;
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
    car->left_target_mm_s = 0;
    car->right_target_mm_s = 0;
}

void car_fail(car_t *car, car_fault_t fault)
{
    car_stop(car);
    car->state = CAR_FAULT;
    car->fault = fault;
}

/* 切换到一段新路时，从这里开始重新记录里程。 */
static void begin_segment(car_t *car)
{
    const road_t *road = current_road(car);
    float heading_difference = angle_error(road->heading - car->yaw_rad);
    car->heading_rad = car->yaw_rad + heading_difference;
    car->arc_end_yaw_rad = car->heading_rad + road->turn * PI_F;
    car->distance_mm = 0;
    car->projected_mm = 0;
    car->crosstrack_mm = 0;
    car->segment_ms = 0;
    car->lost_line_ms = 0;
    car->line_seen = false;
    car->line_previous_valid = false;
    car->last_line_error = 0;

    if (fabsf(heading_difference) > TURN_TOLERANCE) {
        car->state = CAR_TURN;
    } else if (road->type == ROAD_STRAIGHT) {
        car->state = CAR_STRAIGHT;
    } else {
        car->state = CAR_ARC;
    }
}

bool car_start(car_t *car, uint8_t task, bool imu_ok)
{
    if (car_running(car)) return false;
    car_geometry_t geometry = car->geometry;
    car_init(car, geometry);
    car->task = task;

    if (!car_geometry_valid(car)) {
        car_fail(car, CAR_ERR_CONFIG);
        return false;
    }
    if (!imu_ok) {
        car_fail(car, CAR_ERR_IMU);
        return false;
    }

    car->point_events = 1; /* 启动时在 A 点提示一次。 */
    begin_segment(car);
    return true;
}

static bool is_final_road(const car_t *car)
{
    if (car->task == 1) return true;
    return car->segment == 3 && car->laps_done + 1U == target_laps(car);
}

static void reach_point(car_t *car)
{
    car->point = current_road(car)->end_point;
    ++car->point_events;
    ++car->segment;

    if (car->segment == 4) {
        car->segment = 0;
        ++car->laps_done;
    }

    if (car->task == 1 || car->laps_done == target_laps(car)) {
        car_stop(car);
        car->state = CAR_DONE;
    } else {
        begin_segment(car);
    }
}

/* turn_speed 是左右轮速差的一半，正数让小车左转。
 * 例：speed=200，turn_speed=30 -> 左轮170，右轮230 mm/s。
 */
static void set_wheel_speeds(car_t *car, float speed, float turn_speed)
{
    float max_turn = speed * 0.92f;
    if (turn_speed > max_turn) turn_speed = max_turn;
    if (turn_speed < -max_turn) turn_speed = -max_turn;

    float left = speed - turn_speed;
    float right = speed + turn_speed;
    float faster_wheel = fmaxf(left, right);
    if (faster_wheel > CAR_MAX_WHEEL_SPEED_MM_S) {
        float scale = CAR_MAX_WHEEL_SPEED_MM_S / faster_wheel;
        left *= scale;
        right *= scale;
    }
    car->left_target_mm_s = left;
    car->right_target_mm_s = right;
}

/* 下面三个函数只负责“正常情况下怎么走”。 */
static void control_turn(car_t *car)
{
    float error = angle_error(car->heading_rad - car->yaw_rad);
    float turn_speed = CAR_TURN_SPEED_MM_S / 1.1f;
    if (error < 0) turn_speed = -turn_speed;
    set_wheel_speeds(car, CAR_TURN_SPEED_MM_S, turn_speed);
}

static void control_straight(car_t *car, const car_input_t *input, const road_t *road)
{
    float remaining_mm = road->length - car->projected_mm;
    if (is_final_road(car)) remaining_mm -= CAR_STOP_LEAD_MM;

    if (remaining_mm <= 0) {
        reach_point(car);
        return;
    }

    float speed = CAR_STRAIGHT_SPEED_MM_S;
    if (remaining_mm < 120) {
        /* 最后 120mm 逐渐减速，接近顶点时降到 80mm/s。 */
        speed = 80.0f + (speed - 80.0f) * remaining_mm / 120.0f;
    }

    float heading_error = angle_error(car->heading_rad - car->yaw_rad);
    float turn_speed = CAR_HEADING_KP * heading_error
                     - CAR_CROSSTRACK_KP * car->crosstrack_mm
                     - CAR_YAW_DAMPING * input->yaw_rate_rad_s;
    set_wheel_speeds(car, speed, turn_speed);
}

static float arc_remaining_angle(const car_t *car, const road_t *road)
{
    return road->turn * (car->arc_end_yaw_rad - car->yaw_rad);
}

static void control_arc(car_t *car, const car_input_t *input, const road_t *road)
{
    float remaining_angle = arc_remaining_angle(car, road);
    float stop_angle = 0;
    if (is_final_road(car)) stop_angle = CAR_STOP_LEAD_MM / ARC_RADIUS_MM;

    bool near_end = remaining_angle <= 0.015f + stop_angle;
    bool travelled_enough = car->distance_mm > road->length * 0.70f;
    if (near_end && travelled_enough && car->line_seen) {
        reach_point(car);
        return;
    }

    float speed = CAR_ARC_SPEED_MM_S;
    if (remaining_angle < 0.25f) speed = 100.0f;

    /* 先按半径 400mm 给出基础轮速差，再用灰度误差修正。 */
    float turn_speed = road->turn * speed * car->geometry.track_mm / (2.0f * ARC_RADIUS_MM);
    if (input->line.valid) {
        float error_change = 0;
        if (car->line_previous_valid) {
            error_change = input->line.error - car->last_line_error;
        }
        turn_speed += CAR_LINE_KP * input->line.error + CAR_LINE_KD * error_change;
        car->last_line_error = input->line.error;
    }
    car->line_previous_valid = input->line.valid;
    set_wheel_speeds(car, speed, turn_speed);
}

/* 运行保护集中在这里。正常控制函数不再重复检查同样的条件。 */
static bool check_running_conditions(car_t *car, const car_input_t *input, const road_t *road)
{
    const uint32_t task_timeout_ms[4] = {30000, 60000, 80000, 240000};
    if (car->elapsed_ms >= task_timeout_ms[car->task - 1]) {
        car_fail(car, CAR_ERR_TIMEOUT);
        return false;
    }
    if (car->state == CAR_TURN && car->segment_ms > 3000) {
        car_fail(car, CAR_ERR_TIMEOUT);
        return false;
    }

    const int32_t counts[2] = {input->left_count, input->right_count};
    const float targets[2] = {car->left_target_mm_s, car->right_target_mm_s};
    for (unsigned wheel = 0; wheel < 2; ++wheel) {
        if (counts[wheel] < 0) car->reversed_ms[wheel] += CONTROL_PERIOD_MS;
        if (counts[wheel] > 0) car->reversed_ms[wheel] = 0;

        if (targets[wheel] > 60 && counts[wheel] == 0) {
            car->stalled_ms[wheel] += CONTROL_PERIOD_MS;
        } else {
            car->stalled_ms[wheel] = 0;
        }
        if (car->reversed_ms[wheel] >= 60) {
            car_fail(car, CAR_ERR_DIRECTION);
            return false;
        }
        if (car->stalled_ms[wheel] >= 1200) {
            car_fail(car, CAR_ERR_STALL);
            return false;
        }
    }

    if (car->state == CAR_STRAIGHT) {
        if (car->distance_mm > road->length + 300 || fabsf(car->crosstrack_mm) > 180) {
            car_fail(car, CAR_ERR_DISTANCE);
            return false;
        }
    }
    if (car->state == CAR_ARC) {
        if (car->distance_mm > road->length * 1.45f) {
            car_fail(car, CAR_ERR_DISTANCE);
            return false;
        }
        if (input->line.valid) {
            car->line_seen = true;
            car->lost_line_ms = 0;
            return true;
        }

        car->lost_line_ms += CONTROL_PERIOD_MS;
        /* 前置探头比轮轴先离开黑线，允许在终点附近补走最后一小段。 */
        float tail_angle = CAR_SENSOR_FORWARD_MM / ARC_RADIUS_MM + 0.10f;
        bool near_end = arc_remaining_angle(car, road) < tail_angle;
        bool travelled_enough = car->distance_mm > road->length * 0.70f;
        if (car->line_seen && near_end && travelled_enough) return true;

        unsigned allowed_lost_ms = 800; /* 接入时最多找线 800ms */
        if (car->line_seen) allowed_lost_ms = 150; /* 循线途中丢线 150ms 停车 */
        if (car->lost_line_ms >= allowed_lost_ms) {
            car_fail(car, CAR_ERR_LINE);
            return false;
        }
    }
    return true;
}

/* 整个路线控制每 10ms 执行这里一次：更新位置 -> 检查停车条件 -> 控制当前动作。 */
void car_tick(car_t *car, const car_input_t *input)
{
    if (!input->imu_ok) {
        if (car_running(car)) car_fail(car, CAR_ERR_IMU);
        return;
    }

    float angle_change = input->yaw_rate_rad_s * CONTROL_DT;
    float middle_angle = car->yaw_rad + angle_change / 2;
    car->yaw_rad += angle_change; /* 角速度 × 时间 = 这 10ms 转过的角度 */
    if (!car_running(car)) return; /* 待机时仍更新角度，方便手转检查 */

    const road_t *road = current_road(car);
    float left_mm = input->left_count * car->geometry.left_mm_per_count;
    float right_mm = input->right_count * car->geometry.right_mm_per_count;
    float forward_mm = (left_mm + right_mm) / 2;
    float relative_angle = middle_angle - car->heading_rad;
    car->distance_mm += forward_mm;
    car->projected_mm += forward_mm * cosf(relative_angle);
    car->crosstrack_mm += forward_mm * sinf(relative_angle);
    car->elapsed_ms += CONTROL_PERIOD_MS;
    car->segment_ms += CONTROL_PERIOD_MS;

    if (car->state == CAR_TURN) {
        float error = angle_error(car->heading_rad - car->yaw_rad);
        if (fabsf(error) <= TURN_TOLERANCE) {
            if (road->type == ROAD_STRAIGHT) car->state = CAR_STRAIGHT;
            else car->state = CAR_ARC;
        }
    }
    if (!check_running_conditions(car, input, road)) return;

    switch (car->state) {
        case CAR_TURN:     control_turn(car); break;
        case CAR_STRAIGHT: control_straight(car, input, road); break;
        case CAR_ARC:      control_arc(car, input, road); break;
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
