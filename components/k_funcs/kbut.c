#include "kbut.h"

#include <stdbool.h>

#include "codecFuc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/task.h"

static const char *TAG = "kbut";
static kbut_config_t s_button_cfg = KBUT_DEFAULT_CONFIG();


esp_err_t button_init(const kbut_config_t *config)
{
    if (config != NULL) {
        s_button_cfg = *config;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << s_button_cfg.button_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "button gpio config failed");


    ESP_LOGI(TAG, "Record button ready on GPIO%d", s_button_cfg.button_gpio);
    return ESP_OK;
}

void button_task(void *pvParameters)
{
    (void)pvParameters;
    bool last_pressed = false;

    while (true) {
        bool pressed = (gpio_get_level(s_button_cfg.button_gpio) == 0);

        if (pressed && !last_pressed) {
            vTaskDelay(s_button_cfg.debounce_ticks);

            if (gpio_get_level(s_button_cfg.button_gpio) == 0) {
                ESP_LOGI(TAG, "button pressed, record owner voice");

                if (audio_record_owner_voice()) {
                    ESP_LOGI(TAG, "owner voice saved");
                } else {
                    ESP_LOGW(TAG, "owner voice record failed");
                }

                while (gpio_get_level(s_button_cfg.button_gpio) == 0) {
                    vTaskDelay(s_button_cfg.poll_ticks);
                }
            }
        }

        last_pressed = pressed;
        vTaskDelay(s_button_cfg.poll_ticks);
    }
}



esp_err_t servo_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
    return ESP_OK;
}

static uint32_t servo_angle_to_duty(int angle)
{
    if (angle < 0) {
        angle = 0;
    }
    if (angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE;
    }

    uint32_t pulse_us = SERVO_MIN_US +
        (SERVO_MAX_US - SERVO_MIN_US) * angle / SERVO_MAX_ANGLE;

    return pulse_us * DUTY_MAX / PWM_PERIOD_US;
}

void servo_set_angle(int angle)
{
    uint32_t duty = servo_angle_to_duty(angle);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    printf("Set servo angle: %d deg, duty: %lu\r\n",
           angle,
           (unsigned long)duty);
}

void servo_feedMiao_once(float feed_time_s)
{
    if (feed_time_s <= 0.0f) {
        feed_time_s = AUTO_FEED_TIME_S;
    }

    (void)audio_play_owner_voice();

    printf("Feed once start, hold %.2f s\r\n", (double)feed_time_s);
    servo_set_angle(SERVO_FEED_ANGLE);
    vTaskDelay(pdMS_TO_TICKS((uint32_t)(feed_time_s * 1000.0f)));
    servo_set_angle(SERVO_IDLE_ANGLE);
    printf("Feed once done\r\n");
}

void servo_auto_feed_task(void *arg)
{
    (void)arg;
    const uint32_t interval_min = AUTO_FEED_INTERVAL_MIN;
    const TickType_t interval_ticks = pdMS_TO_TICKS(interval_min * 60 * 1000UL);

    printf("Auto feed task started, interval: %lu min\r\n", (unsigned long)interval_min);

    while (1) {
        vTaskDelay(interval_ticks);
        servo_feedMiao_once(AUTO_FEED_TIME_S);
    }
}