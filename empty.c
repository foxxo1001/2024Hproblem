/* 2024 H 题：双电机差速小车。
 * 读代码先找下面的 PID_TIMER_INST_IRQHandler，那里是每 10ms 的完整流程。
 * 再读 car.c 中的 car_tick，最后读 drive.c 和 pid.c。
 */
#include "ti_msp_dl_config.h"
#include "app_config.h"
#include "car.h"
#include "drive.h"
#include "encoder.h"
#include "key.h"
#include "line_sensor.h"
#include "motor.h"
#include "mpu6050.h"
#include "uart.h"
#include "oled.h"
#include <stdio.h>

static car_t g_car;                 /* 路线、方向、里程和目标轮速 */
static drive_t g_drive;             /* 左右轮的 PID 和 PWM */
static car_input_t g_input;         /* 本周期读到的传感器数据 */
static volatile uint32_t g_time_ms;
static bool g_imu_ok;
static bool g_geometry_ready;
static uint8_t g_selected_task = 1;
static uint8_t g_last_point_events;
static uint16_t g_beep_ms;          /* 蜂鸣器还要响多久 */
static uint16_t g_brake_ms;         /* 电机制动还要保持多久 */

static void read_sensors(void)
{
    encoder_read_pair(&g_input.left_count, &g_input.right_count);
    g_input.line = line_sensor_read();
    if (g_imu_ok) {
        g_imu_ok = mpu6050_read_yaw_rate(&g_input.yaw_rate_rad_s);
    }
    g_input.imu_ok = g_imu_ok;
}

static void handle_keys(void)
{
    uint8_t pressed = key_scan_pressed();
    bool key1_pressed = (pressed & 1U) != 0;
    bool key2_pressed = (pressed & 2U) != 0;

    if (key2_pressed) {
        if (car_running(&g_car)) {
            car_stop(&g_car);
            g_beep_ms = 120;
        } else {
            car_stop(&g_car);
            ++g_selected_task;
            if (g_selected_task > 4) g_selected_task = 1;
        }
    } else if (key1_pressed && !car_running(&g_car)) {
        drive_stop(&g_drive, false);
        g_brake_ms = 0;
        bool started = car_start(&g_car, g_selected_task, g_imu_ok);
        g_last_point_events = g_car.point_events;
        if (started) g_beep_ms = 120;
    }
}

static void update_motors(bool was_running)
{
    if (car_running(&g_car)) {
        drive_update(&g_drive, &g_car, &g_input);
        g_brake_ms = 0;
    } else if (was_running) {
        /* 刚停下：立即清 PID 并刹车，不经过减速斜坡。 */
        drive_stop(&g_drive, true);
        g_brake_ms = 300;
    } else if (g_brake_ms > 0) {
        g_brake_ms -= CONTROL_PERIOD_MS;
        if (g_brake_ms == 0) motor_disable();
    }
}

static void update_indicators(void)
{
    if (g_car.point_events != g_last_point_events) {
        g_last_point_events = g_car.point_events;
        g_beep_ms = 120;
    }

    bool fault_flash = g_car.state == CAR_FAULT && g_time_ms % 500 < 100;
    bool signal_on = g_beep_ms > 0 || fault_flash;
    if (signal_on) DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);
    else DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);

    if (car_running(&g_car) || fault_flash) DL_GPIO_setPins(LED_PORT, LED_LED1_PIN);
    else DL_GPIO_clearPins(LED_PORT, LED_LED1_PIN);

    if ((unsigned)signal_on == BUZZER_ACTIVE_LEVEL) {
        DL_GPIO_setPins(ALARM_PORT, ALARM_BUZZER_PIN);
    } else {
        DL_GPIO_clearPins(ALARM_PORT, ALARM_BUZZER_PIN);
    }
    if (g_beep_ms > 0) g_beep_ms -= CONTROL_PERIOD_MS;
}

/* 定时器每 10ms 进入一次。显示、串口不放这里，避免占用控制时间。 */
void PID_TIMER_INST_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(PID_TIMER_INST) != DL_TIMER_IIDX_ZERO) return;
    g_time_ms += CONTROL_PERIOD_MS;
    bool was_running = car_running(&g_car);

    read_sensors();                    /* 1. 测轮速、灰度和角速度 */
    handle_keys();                     /* 2. 选项目、启动或停车 */
    car_tick(&g_car, &g_input);         /* 3. 决定当前动作和左右目标轮速 */
    update_motors(was_running);        /* 4. 双轮 PID -> PWM，或执行停车 */
    update_indicators();              /* 5. 经过顶点时响蜂鸣器、亮灯 */
}

#if CAR_USE_OLED
static void show_oled_row(unsigned y, const char *text)
{
    OLED_ShowString(0, y, (u8 *)"                ", 16);
    OLED_ShowString(0, y, (u8 *)text, 16);
}
#endif

/* 每 500ms 显示一次，只查看数据，不参与控制。 */
static void show_status(void)
{
    car_t car;
    car_input_t input;
    uint16_t left_pwm, right_pwm;
    uint8_t selected_task;

    /* 先复制一份数据，避免显示到一半时定时器改了数据。
     * 暂停中断只覆盖这几次赋值；真正刷屏时中断仍正常运行。
     */
    uint32_t saved_interrupt_state = __get_PRIMASK();
    __disable_irq();
    car = g_car;
    input = g_input;
    left_pwm = g_drive.left_pwm;
    right_pwm = g_drive.right_pwm;
    selected_task = g_selected_task;
    __set_PRIMASK(saved_interrupt_state);

    int32_t left_total, right_total;
    encoder_total_pair(&left_total, &right_total);
    const char *error_text = car_fault_name(car.fault);
    if (car.fault == CAR_OK && !g_geometry_ready) error_text = "CFG";

    char text[200];
    snprintf(text, sizeof(text),
        "M:%u state:%s err:%s imu:%u seg:%u lap:%u point:%c t:%lu "
        "cnt:%ld,%ld total:%ld,%ld pwm:%u,%u gray:%02X yaw10:%ld\r\n",
        (unsigned)selected_task, car_state_name(car.state), error_text, (unsigned)input.imu_ok,
        (unsigned)car.segment, (unsigned)car.laps_done, car.point,
        (unsigned long)car.elapsed_ms, (long)input.left_count, (long)input.right_count,
        (long)left_total, (long)right_total, (unsigned)left_pwm, (unsigned)right_pwm,
        (unsigned)input.line.mask, (long)(car.yaw_rad * 572.957795f));
    UART_SendString(text);

#if CAR_USE_OLED
    char row[17];
    snprintf(row, sizeof(row), "M%u %-8s %s", (unsigned)selected_task,
        car_state_name(car.state), error_text);
    show_oled_row(0, row);
    snprintf(row, sizeof(row), "E%ld %ld", (long)left_total, (long)right_total);
    show_oled_row(16, row);
    snprintf(row, sizeof(row), "G%02X Y%+5ld", (unsigned)input.line.mask,
        (long)(car.yaw_rad * 57.2957795f));
    show_oled_row(32, row);
    snprintf(row, sizeof(row), "L%u P%c T%lu", (unsigned)car.laps_done,
        car.point, (unsigned long)(car.elapsed_ms / 1000));
    show_oled_row(48, row);
    OLED_Refresh();
#endif
}

int main(void)
{
    /* 1. 初始化外设。控制定时器暂时不开，车保持静止。 */
    __disable_irq();
    SYSCFG_DL_init();
    motor_init();
    drive_init(&g_drive);

    /* 2. 根据实测尺寸，把“编码器计数”换算成“毫米”。 */
    car_geometry_t geometry = {0};
    float wheel_circumference = 3.14159265359f * CAR_WHEEL_DIAMETER_MM;
    if (CAR_LEFT_COUNTS_PER_WHEEL_REV > 0) {
        geometry.left_mm_per_count = wheel_circumference / CAR_LEFT_COUNTS_PER_WHEEL_REV;
    }
    if (CAR_RIGHT_COUNTS_PER_WHEEL_REV > 0) {
        geometry.right_mm_per_count = wheel_circumference / CAR_RIGHT_COUNTS_PER_WHEEL_REV;
    }
    geometry.track_mm = CAR_WHEEL_TRACK_MM;
    car_init(&g_car, geometry);
    g_geometry_ready = car_geometry_valid(&g_car);
    update_indicators();

    /* 3. 静止约 3 秒，测 MPU6050 的零偏。 */
#if CAR_USE_OLED
    OLED_Init();
    show_oled_row(0, "H2024 KEEP STILL");
    show_oled_row(16, "IMU CALIBRATING");
    OLED_Refresh();
#endif
    UART_SendString("H2024: keep still for MPU6050 calibration...\r\n");
    g_imu_ok = mpu6050_init_and_calibrate();
    if (g_imu_ok) {
        UART_SendString("IMU ready\r\n");
    } else {
        car_fail(&g_car, CAR_ERR_IMU);
        UART_SendString("IMU failed: check wiring/stillness, then reset\r\n");
    }
    if (!g_geometry_ready) {
        UART_SendString("CFG: fill wheel diameter, counts/rev and track in app_config.h\r\n");
    }

    /* 4. 开编码器和 10ms 控制定时器，之后由中断负责行驶。 */
    encoder_init();
    NVIC_SetPriority(PID_TIMER_INST_INT_IRQN, 2);
    DL_Timer_clearInterruptStatus(PID_TIMER_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(PID_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(PID_TIMER_INST_INT_IRQN);
    DL_Timer_startCounter(PID_TIMER_INST);
    __enable_irq();

    /* 5. 主循环只负责显示，其余时间等待中断。 */
    uint32_t last_display_ms = 0;
    while (1) {
        if ((uint32_t)(g_time_ms - last_display_ms) >= 500) {
            last_display_ms = g_time_ms;
            show_status();
        }
        __WFI();
    }
}
