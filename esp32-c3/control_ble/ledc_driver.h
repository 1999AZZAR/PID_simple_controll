#ifndef LEDC_DRIVER_H
#define LEDC_DRIVER_H

#include "driver/ledc.h"

// LEDC Configuration
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT // 8-bit resolution (0-255) to match logic
#define LEDC_FREQUENCY 5000            // 5 kHz PWM (High frequency for smoother ESC operation)

// Fix #1: marked inline to avoid ODR violations when included in multiple TUs
inline void ledc_init(int gpio_num) {
    ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_MODE,
                                      .duty_resolution = LEDC_DUTY_RES,
                                      .timer_num = LEDC_TIMER,
                                      .freq_hz = LEDC_FREQUENCY,
                                      .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {.gpio_num = gpio_num,
                                          .speed_mode = LEDC_MODE,
                                          .channel = LEDC_CHANNEL,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .timer_sel = LEDC_TIMER,
                                          .duty = 0,
                                          .hpoint = 0};
    ledc_channel_config(&ledc_channel);
}

// Fix #1 + #2: renamed from ledc_set_duty() to motor_set_pwm() to avoid shadowing
// the ESP-IDF ledc_set_duty(ledc_mode_t, ledc_channel_t, uint32_t) function,
// and marked inline to avoid ODR violations.
inline void motor_set_pwm(int duty) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

#endif // LEDC_DRIVER_H
