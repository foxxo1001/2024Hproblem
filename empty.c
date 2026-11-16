/* 基础版：10ms 读传感器 -> 路线控制 -> 双轮速度 PID -> PWM。 */
#include "ti_msp_dl_config.h"
#include "app_config.h"
#include "car.h"
#include "encoder.h"
#include "key.h"
#include "line_sensor.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid.h"
#include "uart.h"
#include "oled.h"
#include <stdio.h>

static car_t car;
static car_input_t input;
static pid_ctrl_t left_pid, right_pid;
static uint16_t left_pwm, right_pwm;
static volatile uint32_t time_ms;
static bool imu_ok;
static uint8_t selected_task = 1, last_point_events;
static uint16_t beep_ms, brake_ms;

static void update_motors(bool was_running)
{
    if (car_running(&car)) {
        /* 目标 mm/s 换成计数/10ms，再与编码器反馈比较。 */
        float left_target = car.left_target_mm_s * CONTROL_DT / car.geometry.left_mm_per_count;
        float right_target = car.right_target_mm_s * CONTROL_DT / car.geometry.right_mm_per_count;
        left_pwm = (uint16_t)PID_calc(&left_pid, input.left_count, left_target);
        right_pwm = (uint16_t)PID_calc(&right_pid, input.right_count, right_target);
        motor_set_forward(left_pwm, right_pwm);
    } else if (was_running) {
        PID_clear(&left_pid);
        PID_clear(&right_pid);
        left_pwm = right_pwm = 0;
        motor_brake();
        brake_ms = 300;
    } else if (brake_ms > 0) {
        brake_ms -= CONTROL_PERIOD_MS;
        if (brake_ms == 0) motor_disable();
    }
}

static void update_indicators(void)
{
    if (car.point_events != last_point_events) {
        last_point_events = car.point_events;
        beep_ms = 120;
    }
    bool signal = beep_ms > 0 || (car.state == CAR_FAULT && time_ms % 500 < 100);
    if (signal) DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);
    else DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
    if (car_running(&car)) DL_GPIO_setPins(LED_PORT, LED_LED1_PIN);
    else DL_GPIO_clearPins(LED_PORT, LED_LED1_PIN);
    if ((unsigned)signal == BUZZER_ACTIVE_LEVEL) DL_GPIO_setPins(ALARM_PORT, ALARM_BUZZER_PIN);
    else DL_GPIO_clearPins(ALARM_PORT, ALARM_BUZZER_PIN);
    if (beep_ms > 0) beep_ms -= CONTROL_PERIOD_MS;
}

void PID_TIMER_INST_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(PID_TIMER_INST) != DL_TIMER_IIDX_ZERO) return;
    time_ms += CONTROL_PERIOD_MS;
    bool was_running = car_running(&car);

    /* 1. 读取编码器、灰度、陀螺仪。 */
    encoder_read_pair(&input.left_count, &input.right_count);
    input.line = line_sensor_read();
    if (imu_ok) imu_ok = mpu6050_read_yaw_rate(&input.yaw_rate_rad_s);
    input.imu_ok = imu_ok;

    /* 2. KEY1 启动；KEY2 待机选路线、运行时停车。 */
    uint8_t key = key_scan_pressed();
    if (key & 2U) {
        car_stop(&car);
        if (!was_running) selected_task = selected_task % 4 + 1;
    } else if ((key & 1U) && !was_running) {
        PID_clear(&left_pid);
        PID_clear(&right_pid);
        motor_disable();
        brake_ms = 0;
        car_start(&car, selected_task, imu_ok);
        last_point_events = car.point_events;
        if (car_running(&car)) beep_ms = 120;
    }

    /* 3. 更新路线，算目标轮速，然后由 PID 输出 PWM。 */
    car_tick(&car, &input);
    update_motors(was_running);
    update_indicators();
}

static void show_row(unsigned y, const char *text)
{
    OLED_ShowString(0, y, (u8 *)"                ", 16);
    OLED_ShowString(0, y, (u8 *)text, 16);
}

static void show_status(void)
{
    /* 只在复制数据时暂停中断，串口和刷屏期间仍能正常测速。 */
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    car_t status = car;
    car_input_t sensors = input;
    uint16_t lpwm = left_pwm, rpwm = right_pwm;
    uint8_t task = selected_task;
    __set_PRIMASK(saved);
    int32_t left_total, right_total;
    encoder_total_pair(&left_total, &right_total);
    const char *fault = car_fault_name(status.fault);
    if (status.fault == CAR_OK && !car_geometry_valid(&status)) fault = "CFG";

    char text[180];
    snprintf(text, sizeof(text),
        "M:%u %s %s cnt:%ld,%ld total:%ld,%ld pwm:%u,%u gray:%02X yaw10:%ld\r\n",
        (unsigned)task, car_state_name(status.state), fault,
        (long)sensors.left_count, (long)sensors.right_count, (long)left_total, (long)right_total,
        (unsigned)lpwm, (unsigned)rpwm, (unsigned)sensors.line.mask,
        (long)(status.yaw_rad * 572.957795f));
    UART_SendString(text);

    char row[17];
    snprintf(row, sizeof(row), "M%u %-8s %s", (unsigned)task, car_state_name(status.state), fault);
    show_row(0, row);
    snprintf(row, sizeof(row), "E%ld %ld", (long)left_total, (long)right_total);
    show_row(16, row);
    snprintf(row, sizeof(row), "G%02X Y%+5ld", (unsigned)sensors.line.mask,
        (long)(status.yaw_rad * 57.2957795f));
    show_row(32, row);
    snprintf(row, sizeof(row), "L%u P%c T%lu", (unsigned)status.laps_done, status.point,
        (unsigned long)(status.elapsed_ms / 1000));
    show_row(48, row);
    OLED_Refresh();
}

int main(void)
{
    __disable_irq();
    SYSCFG_DL_init();
    motor_init();
    PID_init(&left_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, MOTOR_PWM_LIMIT, SPEED_PID_IMAX);
    PID_init(&right_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, MOTOR_PWM_LIMIT, SPEED_PID_IMAX);

    car_geometry_t geometry = {0};
    float circumference = 3.14159265359f * CAR_WHEEL_DIAMETER_MM;
    if (CAR_LEFT_COUNTS_PER_WHEEL_REV > 0)
        geometry.left_mm_per_count = circumference / CAR_LEFT_COUNTS_PER_WHEEL_REV;
    if (CAR_RIGHT_COUNTS_PER_WHEEL_REV > 0)
        geometry.right_mm_per_count = circumference / CAR_RIGHT_COUNTS_PER_WHEEL_REV;
    geometry.track_mm = CAR_WHEEL_TRACK_MM;
    car_init(&car, geometry);

    OLED_Init();
    show_row(0, "KEEP STILL");
    show_row(16, "IMU CALIBRATING");
    OLED_Refresh();
    UART_SendString("Keep still for MPU6050 calibration...\r\n");
    imu_ok = mpu6050_init_and_calibrate();
    if (!imu_ok) car_fail(&car, CAR_ERR_IMU);
    UART_SendString(imu_ok ? "IMU ready\r\n" : "IMU failed: check wiring and reset\r\n");

    encoder_init();
    NVIC_SetPriority(PID_TIMER_INST_INT_IRQN, 2);
    DL_Timer_clearInterruptStatus(PID_TIMER_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(PID_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(PID_TIMER_INST_INT_IRQN);
    DL_Timer_startCounter(PID_TIMER_INST);
    __enable_irq();

    uint32_t last_display = 0;
    while (1) {
        if ((uint32_t)(time_ms - last_display) >= 500) {
            last_display = time_ms;
            show_status();
        }
        __WFI();
    }
}
