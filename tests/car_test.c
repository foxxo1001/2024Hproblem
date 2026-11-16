#include "car.h"
#include "pid.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI_D 3.14159265358979323846
static const car_geometry_t geometry = {0.25f, 0.25f, 120.0f};
static void basic_tests(void)
{
    assert(!line_decode(0x00).valid);
    assert(!line_decode(0xFF).valid);
    assert(!line_decode(0x81).valid);
    assert(line_decode(0x18).valid && line_decode(0x18).error == 0.0f);
    assert(line_decode(0x01).error > 0.0f);
    assert(line_decode(0x80).error < 0.0f);
    car_t car;
    car_init(&car, (car_geometry_t){0});
    assert(!car_start(&car, 1, true) && car.fault == CAR_ERR_CONFIG);
    car_init(&car, geometry);
    assert(!car_start(&car, 1, false) && car.fault == CAR_ERR_IMU);
    assert(car_start(&car, 1, true));
    assert(!car_start(&car, 2, true));
    car_input_t in = {.imu_ok = false};
    car_tick(&car, &in);
    assert(car.fault == CAR_ERR_IMU && car.left_target_mm_s == 0);
    car_stop(&car);
    assert(car.state == CAR_IDLE && car.right_target_mm_s == 0);
    assert(car_start(&car, 1, true));
    in = (car_input_t){.imu_ok = true, .left_count = -1, .right_count = -1};
    for (int i = 0; i < 6; ++i) car_tick(&car, &in);
    assert(car.fault == CAR_ERR_DIRECTION);
    assert(car_start(&car, 1, true));
    in = (car_input_t){.imu_ok = true};
    for (int i = 0; i < 125; ++i) car_tick(&car, &in);
    assert(car.fault == CAR_ERR_STALL);
    assert(car_start(&car, 1, true));
    car.elapsed_ms = 29990;
    car_tick(&car, &in);
    assert(car.fault == CAR_ERR_TIMEOUT);

    pid_ctrl_t pid;
    PID_init(&pid, 10, 2, 0, 100, 80);
    for (int i = 0; i < 1000; ++i) {
        float out = PID_calc(&pid, 0, 100);
        assert(out == 100 && pid.i_out == 0);
    }
    assert(PID_calc(&pid, 100, 0) == 0 && pid.i_out == 0);
    PID_clear(&pid);
    assert(PID_calc(&pid, 5, 0) == 0); /* 超速时降 PWM，不反转 */
}

/* 独立的场地几何模型，实际黑线只在左右半圆上，直线全白。 */
static line_sample_t field_line(double x, double y, double yaw)
{
    uint8_t mask = 0;
    for (unsigned i = 0; i < 8; ++i) {
        double lateral = (3.5 - i) * 10.0;
        double sx = x + CAR_SENSOR_FORWARD_MM * cos(yaw) - lateral * sin(yaw);
        double sy = y + CAR_SENSOR_FORWARD_MM * sin(yaw) + lateral * cos(yaw);
        bool left = sx <= 0.0 && fabs(hypot(sx, sy + 400.0) - 400.0) <= 9.0;
        bool right = sx >= 1000.0 && fabs(hypot(sx - 1000.0, sy + 400.0) - 400.0) <= 9.0;
        if (left || right) mask |= (uint8_t)(1U << i);
    }
    return line_decode(mask);
}

static void simulate(unsigned task, unsigned remove_line, bool slew_speed)
{
    car_t car;
    car_init(&car, geometry);
    assert(car_start(&car, (uint8_t)task, true));
    double x = 0, y = 0;
    double yaw = task >= 3 ? -atan2(800.0, 1000.0) : 0;
    double left_fraction = 0, right_fraction = 0, mean_speed = 0;
    double actual_left = 0, actual_right = 0;
    double max_point_error = 0;
    unsigned events = car.point_events;
    char visited[32] = "A";
    unsigned visited_length = 1;
    for (unsigned tick = 0; tick < 24000 && car_running(&car); ++tick) {
        assert(car.left_target_mm_s >= 0 && car.right_target_mm_s >= 0);
        assert(car.left_target_mm_s <= CAR_MAX_WHEEL_SPEED_MM_S + 0.01f);
        assert(car.right_target_mm_s <= CAR_MAX_WHEEL_SPEED_MM_S + 0.01f);
        double left = car.left_target_mm_s, right = car.right_target_mm_s;
        if (slew_speed) {
            double target = (left + right) * 0.5;
            double step = 450.0 * CONTROL_DT; /* 模拟电机有限的加速度 */
            if (mean_speed < target) mean_speed = fmin(target, mean_speed + step);
            else mean_speed = fmax(target, mean_speed - step);
            if (target > 0) { left *= mean_speed / target; right *= mean_speed / target; }
            double peak = fmax(left, right);
            if (peak > CAR_MAX_WHEEL_SPEED_MM_S) {
                left *= CAR_MAX_WHEEL_SPEED_MM_S / peak;
                right *= CAR_MAX_WHEEL_SPEED_MM_S / peak;
                mean_speed = 0.5 * (left + right);
            }
            /* 非瞬时执行器：60ms 一阶速度响应。 */
            actual_left += (left - actual_left) * CONTROL_DT / (0.06 + CONTROL_DT);
            actual_right += (right - actual_right) * CONTROL_DT / (0.06 + CONTROL_DT);
            left = actual_left;
            right = actual_right;
        }
        double ds = (left + right) * 0.5 * CONTROL_DT;
        double da = (right - left) / geometry.track_mm * CONTROL_DT;
        x += ds * cos(yaw + da * 0.5);
        y += ds * sin(yaw + da * 0.5);
        yaw += da;
        left_fraction += left * CONTROL_DT / geometry.left_mm_per_count;
        right_fraction += right * CONTROL_DT / geometry.right_mm_per_count;
        car_input_t in = {
            .left_count = (int32_t)floor(left_fraction),
            .right_count = (int32_t)floor(right_fraction),
            .yaw_rate_rad_s = (float)(da / CONTROL_DT), .imu_ok = true,
            .line = (remove_line == 1U || (remove_line == 2U && car.line_seen && car.distance_mm > 300))
                ? line_decode(0) : field_line(x, y, yaw)
        };
        left_fraction -= in.left_count;
        right_fraction -= in.right_count;
        car_tick(&car, &in);
        if (car.state == CAR_ARC) {
            double center_x = car.segment == 1 ? 1000 : 0;
            assert(fabs(hypot(x - center_x, y + 400) - 400) < 70);
        }
        if (events != car.point_events) {
            events = car.point_events;
            visited[visited_length++] = car.point;
            visited[visited_length] = '\0';
            double px = car.point == 'B' || car.point == 'C' ? 1000 : 0;
            double py = car.point == 'C' || car.point == 'D' ? -800 : 0;
            double error = hypot(x - px, y - py);
            max_point_error = fmax(max_point_error, error);
        }
    }
    printf("task %u slew=%u remove_line=%u: %s/%s visits=%s t=%.2f s max_point_error=%.1f mm\n",
        task, slew_speed, remove_line, car_state_name(car.state), car_fault_name(car.fault),
        visited, car.elapsed_ms / 1000.0, max_point_error);
    if (remove_line) {
        assert(car.state == CAR_FAULT && car.fault == CAR_ERR_LINE);
    } else {
        assert(car.state == CAR_DONE && car.fault == CAR_OK);
        unsigned expected_events = task == 1 ? 2 : (task == 4 ? 17 : 5);
        assert(car.point_events == expected_events);
        static const char *const expected_routes[] = {
            "AB", "ABCDA", "ACBDA", "ACBDACBDACBDACBDA"
        };
        assert(strcmp(visited, expected_routes[task - 1]) == 0);
        assert(car.laps_done == (task == 1 ? 0 : (task == 4 ? 4 : 1)));
        assert(max_point_error < 75.0); /* 小于车身投影的可覆盖范围 */
        assert(car.elapsed_ms < (task == 1 ? 15000U : (task == 2 ? 30000U : (task == 3 ? 40000U : 240000U))));
        assert(car.left_target_mm_s == 0 && car.right_target_mm_s == 0);
    }
}

int main(void)
{
    setbuf(stdout, NULL);
    basic_tests();

    for (unsigned task = 1; task <= 4; ++task) {
        simulate(task, false, false);
        simulate(task, false, true);
    }
    simulate(2, true, true);
    simulate(3, 2, true);
    puts("All basic car tests passed.");
    return 0;
}
