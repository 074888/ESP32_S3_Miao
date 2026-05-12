#include "esp_log.h"
#include "kbut.h"
#include "codecFuc.h"

static const char *TAG = "es8311_capture";

void app_main(void)
{
    ESP_LOGI(TAG, "Start ES8311 + NS4150 UART capture and local loopback");

    ESP_ERROR_CHECK(board_i2c_init());
    ESP_ERROR_CHECK(board_i2s_init());
    ESP_ERROR_CHECK(codec_devices_init());
    ESP_ERROR_CHECK(uart_audio_init());
    ESP_ERROR_CHECK(servo_init());
    ESP_ERROR_CHECK(button_init(NULL));
    ESP_ERROR_CHECK(audio_storage_init());

    s_uart_audio_rb = xRingbufferCreate(UART_AUDIO_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    s_capture_cmd_queue = xQueueCreate(1, sizeof(int));
    s_uart_stream_queue = xQueueCreate(1, sizeof(capture_stream_info_t));
    s_uart_tx_mutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(s_uart_audio_rb != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(s_capture_cmd_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(s_uart_stream_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(s_uart_tx_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    ESP_ERROR_CHECK(xTaskCreate(uart_audio_tx_task,
                                "uart_audio_tx",
                                TASK_STACK_SIZE,
                                NULL,
                                UART_TX_TASK_PRIO,
                                NULL) == pdPASS ? ESP_OK : ESP_FAIL);

    ESP_ERROR_CHECK(xTaskCreate(audio_capture_task,
                                "audio_capture",
                                TASK_STACK_SIZE,
                                NULL,
                                AUDIO_CAPTURE_TASK_PRIO,
                                &s_capture_task_handle) == pdPASS ? ESP_OK : ESP_FAIL);

    ESP_ERROR_CHECK(xTaskCreate(uart_cmd_task,
                                "uart_cmd",
                                TASK_STACK_SIZE,
                                NULL,
                                UART_CMD_TASK_PRIO,
                                NULL) == pdPASS ? ESP_OK : ESP_FAIL);

    ESP_ERROR_CHECK(xTaskCreate(servo_auto_feed_task,
                                "servo_auto_feed",
                                TASK_STACK_SIZE,
                                NULL,
                                UART_CMD_TASK_PRIO,
                                NULL) == pdPASS ? ESP_OK : ESP_FAIL);

    ESP_ERROR_CHECK(xTaskCreate(button_task,
                            "button_task",
                            TASK_STACK_SIZE,
                            NULL,
                            5,
                            NULL) == pdPASS ? ESP_OK : ESP_FAIL);
}
