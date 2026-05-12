#ifndef CODEC_FUC_H_
#define CODEC_FUC_H_

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "utilities.h"

extern RingbufHandle_t s_uart_audio_rb;
extern QueueHandle_t s_capture_cmd_queue;
extern QueueHandle_t s_uart_stream_queue;
extern SemaphoreHandle_t s_uart_tx_mutex;
extern TaskHandle_t s_capture_task_handle;

// static SemaphoreHandle_t s_audio_mutex;

esp_err_t board_i2c_init(void);
esp_err_t board_i2s_init(void);
esp_err_t codec_devices_init(void);
esp_err_t uart_audio_init(void);

esp_err_t codec_request_capture(int duration_sec);
bool codec_capture_is_busy(void);

void uart_audio_tx_task(void *arg);
void audio_capture_task(void *arg);
void uart_cmd_task(void *arg);

esp_err_t audio_storage_init(void);
bool audio_record_owner_voice(void);
bool audio_play_owner_voice(void);
#endif /* CODEC_FUC_H_ */
