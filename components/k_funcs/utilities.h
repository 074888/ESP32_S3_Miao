#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <stddef.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/uart.h"

#define CODEC_I2C_PORT              I2C_NUM_0
#define CODEC_I2C_SCL_GPIO          GPIO_NUM_15
#define CODEC_I2C_SDA_GPIO          GPIO_NUM_16

#define CODEC_I2S_PORT              I2S_NUM_0
#define CODEC_I2S_MCLK_GPIO         GPIO_NUM_0
#define CODEC_I2S_BCLK_GPIO         GPIO_NUM_4
#define CODEC_I2S_WS_GPIO           GPIO_NUM_5
#define CODEC_I2S_DIN_GPIO          GPIO_NUM_7
#define CODEC_I2S_DOUT_GPIO         GPIO_NUM_8

#define CODEC_PA_ENABLE_GPIO        GPIO_NUM_NC

#define AUDIO_SAMPLE_RATE_HZ        16000U
#define AUDIO_CHANNELS              2U
#define AUDIO_BITS_PER_SAMPLE       16U
#define AUDIO_VOLUME_PERCENT        35U
#define AUDIO_IN_GAIN_DB            18.0f
#define AUDIO_SAMPLES_ONECH_CHUNK   256U
#define AUDIO_BYTES_PER_SAMPLE      ((AUDIO_CHANNELS * AUDIO_BITS_PER_SAMPLE) / 8U)
#define AUDIO_BYTES_PER_CHUNK       (AUDIO_SAMPLES_ONECH_CHUNK * AUDIO_BYTES_PER_SAMPLE)

#define UART_AUDIO_PORT             UART_NUM_0
#define UART_AUDIO_BAUD_RATE        921600
#define UART_AUDIO_RINGBUF_SIZE     (64U * 1024U)
#define UART_CMD_MAX_LEN            64U
#define GET_AUDIO_MIN_SECONDS       1U
#define GET_AUDIO_MAX_SECONDS       30U

#define CODEC_CAPTURE_QUEUE_LEN     1U
#define CODEC_STREAM_QUEUE_LEN      1U

#define TASK_STACK_SIZE             4096U
#define CODEC_TASK_STACK_SIZE       TASK_STACK_SIZE
#define UART_CMD_TASK_PRIO          5U
#define AUDIO_CAPTURE_TASK_PRIO     6U
#define UART_TX_TASK_PRIO           7U

#define RECORD_BUTTON_GPIO          GPIO_NUM_13
#define BUTTON_DEBOUNCE_MS          40U
#define BUTTON_POLL_INTERVAL_MS     100U

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY          50
#define SERVO_GPIO              21
#define SERVO_MIN_US            500
#define SERVO_MAX_US            2500
#define SERVO_MAX_ANGLE         180
#define PWM_PERIOD_US           20000
#define DUTY_MAX                ((1 << 13) - 1)
#define SERVO_IDLE_ANGLE        0
#define SERVO_FEED_ANGLE        180
#define AUTO_FEED_INTERVAL_MIN  1
#define AUTO_FEED_TIME_S        0.5f

#define OWNER_VOICE_FILE "/spiffs/owner_voice.pcm"
#define RECORD_DURATION_SEC     3


typedef struct {
    int duration_sec;
    size_t total_bytes;
} capture_stream_info_t;

#endif /* UTILITIES_H_ */
