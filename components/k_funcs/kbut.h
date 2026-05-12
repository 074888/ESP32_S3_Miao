#ifndef KBUT_H_
#define KBUT_H_

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "utilities.h"

typedef struct {
    gpio_num_t button_gpio;
    TickType_t debounce_ticks;
    TickType_t poll_ticks;
    uint8_t capture_duration_sec;
} kbut_config_t;

#define KBUT_DEFAULT_CONFIG()                       \
    {                                              \
        .button_gpio = RECORD_BUTTON_GPIO,         \
        .debounce_ticks = pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS), \
        .poll_ticks = pdMS_TO_TICKS(BUTTON_POLL_INTERVAL_MS), \
        .capture_duration_sec = RECORD_DURATION_SEC, \
    }

esp_err_t button_init(const kbut_config_t *config);
void button_task(void *pvParameters);
void servo_set_angle(int angle);
esp_err_t servo_init(void);
void servo_feedMiao_once(float feed_time_s);
void servo_auto_feed_task(void *arg);
#endif /* KBUT_H_ */
