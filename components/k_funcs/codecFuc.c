#include "codecFuc.h"

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_uart.h"
#include "es8311_codec.h"
#include "esp_spiffs.h"
#include "kbut.h"

static const char *TAG = "codec_fuc";

static i2c_master_bus_handle_t s_i2c_bus;
static i2s_chan_handle_t s_i2s_tx_handle;
static i2s_chan_handle_t s_i2s_rx_handle;

static const audio_codec_data_if_t *s_data_if;
static const audio_codec_ctrl_if_t *s_codec_ctrl_if;
static const audio_codec_gpio_if_t *s_gpio_if;
static const audio_codec_if_t *s_codec_if;

static esp_codec_dev_handle_t s_codec_dev;
static esp_codec_dev_handle_t s_play_dev;
static esp_codec_dev_handle_t s_record_dev;

RingbufHandle_t s_uart_audio_rb;
QueueHandle_t s_capture_cmd_queue;
QueueHandle_t s_uart_stream_queue;
SemaphoreHandle_t s_uart_tx_mutex;
TaskHandle_t s_capture_task_handle;

static portMUX_TYPE s_capture_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_capture_busy;

static void prepare_stereo_output(const int16_t *in_pcm, int16_t *out_pcm, size_t frames)
{
    int64_t ch_energy[AUDIO_CHANNELS] = {0};

    for (size_t i = 0; i < frames; ++i) {
        for (size_t ch = 0; ch < AUDIO_CHANNELS; ++ch) {
            int32_t sample = in_pcm[i * AUDIO_CHANNELS + ch];
            ch_energy[ch] += (int64_t)sample * sample;
        }
    }

    int dominant_ch = (ch_energy[1] > ch_energy[0]) ? 1 : 0;
    int quiet_ch = dominant_ch ^ 1;
    bool use_dominant = ch_energy[quiet_ch] * 8 < ch_energy[dominant_ch];

    for (size_t i = 0; i < frames; ++i) {
        int32_t sample = use_dominant ? in_pcm[i * AUDIO_CHANNELS + dominant_ch]
                                      : (in_pcm[i * AUDIO_CHANNELS] + in_pcm[i * AUDIO_CHANNELS + 1]) / 2;
        out_pcm[i * AUDIO_CHANNELS] = (int16_t)sample;
        out_pcm[i * AUDIO_CHANNELS + 1] = (int16_t)sample;
    }
}

static bool capture_try_acquire(void)
{
    bool acquired = false;

    portENTER_CRITICAL(&s_capture_lock);
    if (!s_capture_busy) {
        s_capture_busy = true;
        acquired = true;
    }
    portEXIT_CRITICAL(&s_capture_lock);

    return acquired;
}

bool codec_capture_is_busy(void)
{
    bool busy = false;

    portENTER_CRITICAL(&s_capture_lock);
    busy = s_capture_busy;
    portEXIT_CRITICAL(&s_capture_lock);

    return busy;
}

static void capture_release(void)
{
    portENTER_CRITICAL(&s_capture_lock);
    s_capture_busy = false;
    portEXIT_CRITICAL(&s_capture_lock);
}

static void log_codec_i2c_probe(void)
{
    const uint16_t expected_addr_7bit = (uint16_t)(ES8311_CODEC_DEFAULT_ADDR >> 1);
    esp_err_t expected_err = i2c_master_probe(s_i2c_bus, expected_addr_7bit, 20);
    esp_err_t raw_err = i2c_master_probe(s_i2c_bus, ES8311_CODEC_DEFAULT_ADDR, 20);

    if (expected_err == ESP_OK) {
        ESP_LOGI(TAG, "I2C probe OK: ES8311 addr7=0x%02X (from raw 0x%02X)", expected_addr_7bit,
                 ES8311_CODEC_DEFAULT_ADDR);
    } else {
        ESP_LOGW(TAG, "I2C probe miss: ES8311 addr7=0x%02X err=%s", expected_addr_7bit, esp_err_to_name(expected_err));
    }

    if (raw_err == ESP_OK) {
        ESP_LOGW(TAG, "I2C probe found ACK on raw-style addr 0x%02X, check board address definition",
                 ES8311_CODEC_DEFAULT_ADDR);
    }
}

static void uart_write_all_unlocked(const void *data, size_t len)
{
    const uint8_t *cursor = (const uint8_t *)data;

    while (len > 0U) {
        esp_rom_output_tx_one_char(*cursor++);
        len--;
    }
}

static void uart_wait_output_idle(void)
{
    esp_rom_output_tx_wait_idle((uint8_t)UART_AUDIO_PORT);
}

static void uart_vprintf_unlocked(const char *fmt, va_list args)
{
    char buffer[192];
    int written = vsnprintf(buffer, sizeof(buffer), fmt, args);
    if (written <= 0) {
        return;
    }

    size_t len = (written < (int)sizeof(buffer)) ? (size_t)written : sizeof(buffer) - 1U;
    uart_write_all_unlocked(buffer, len);
}

static void uart_printf_locked(const char *fmt, ...)
{
    va_list args;

    xSemaphoreTake(s_uart_tx_mutex, portMAX_DELAY);
    va_start(args, fmt);
    uart_vprintf_unlocked(fmt, args);
    va_end(args);
    xSemaphoreGive(s_uart_tx_mutex);
}

static void uart_printf_unlocked(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    uart_vprintf_unlocked(fmt, args);
    va_end(args);
}

static void uart_advertise_ready(void)
{
    uart_printf_locked("READY getAudio_<n>S format=s16le_stereo_16k baud=%d\n", UART_AUDIO_BAUD_RATE);
    uart_wait_output_idle();
}

static bool parse_get_audio_command(const char *cmd, int *seconds)
{
    static const char *prefix = "getAudio_";
    char *end = NULL;
    long duration = 0;

    if (cmd == NULL || seconds == NULL) {
        return false;
    }
    if (strncmp(cmd, prefix, strlen(prefix)) != 0) {
        return false;
    }

    duration = strtol(cmd + strlen(prefix), &end, 10);
    if (end == cmd + strlen(prefix) || duration < GET_AUDIO_MIN_SECONDS || duration > GET_AUDIO_MAX_SECONDS) {
        return false;
    }
    if ((*end != 'S' && *end != 's') || end[1] != '\0') {
        return false;
    }

    *seconds = (int)duration;
    return true;
}

static bool parse_feed_command(const char *cmd)
{
    return (cmd != NULL) && (strcmp(cmd, "feed") == 0);
}

esp_err_t board_i2c_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = CODEC_I2C_PORT,
        .sda_io_num = CODEC_I2C_SDA_GPIO,
        .scl_io_num = CODEC_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&cfg, &s_i2c_bus), TAG, "i2c init failed");
    ESP_LOGI(TAG, "I2C ready: sda=%d scl=%d", CODEC_I2C_SDA_GPIO, CODEC_I2C_SCL_GPIO);
    log_codec_i2c_probe();
    return ESP_OK;
}

esp_err_t board_i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CODEC_I2S_PORT, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_i2s_tx_handle, &s_i2s_rx_handle), TAG,
                        "i2s channel alloc failed");

    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE_HZ);
    clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    i2s_std_config_t std_cfg = {
        .clk_cfg = clk_cfg,
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = CODEC_I2S_MCLK_GPIO,
            .bclk = CODEC_I2S_BCLK_GPIO,
            .ws = CODEC_I2S_WS_GPIO,
            .dout = CODEC_I2S_DOUT_GPIO,
            .din = CODEC_I2S_DIN_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s_tx_handle, &std_cfg), TAG, "i2s tx init failed");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s_rx_handle, &std_cfg), TAG, "i2s rx init failed");

    ESP_LOGI(TAG, "I2S ready: mclk=%d bclk=%d ws=%d din=%d dout=%d", CODEC_I2S_MCLK_GPIO, CODEC_I2S_BCLK_GPIO,
             CODEC_I2S_WS_GPIO, CODEC_I2S_DIN_GPIO, CODEC_I2S_DOUT_GPIO);
    return ESP_OK;
}

esp_err_t codec_devices_init(void)
{
    audio_codec_i2s_cfg_t data_cfg = {
        .port = CODEC_I2S_PORT,
        .rx_handle = s_i2s_rx_handle,
        .tx_handle = s_i2s_tx_handle,
    };
    s_data_if = audio_codec_new_i2s_data(&data_cfg);
    if (s_data_if == NULL) {
        ESP_LOGE(TAG, "audio_codec_new_i2s_data failed");
        return ESP_FAIL;
    }

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = CODEC_I2C_PORT,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = s_i2c_bus,
    };
    s_codec_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (s_codec_ctrl_if == NULL) {
        ESP_LOGE(TAG, "audio_codec_new_i2c_ctrl failed");
        return ESP_FAIL;
    }

    s_gpio_if = audio_codec_new_gpio();
    if (s_gpio_if == NULL) {
        ESP_LOGE(TAG, "audio_codec_new_gpio failed");
        return ESP_FAIL;
    }

    es8311_codec_cfg_t codec_cfg = {
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .ctrl_if = s_codec_ctrl_if,
        .gpio_if = s_gpio_if,
        .pa_pin = CODEC_PA_ENABLE_GPIO,
        .use_mclk = true,
        .no_dac_ref = true,
    };
    s_codec_if = es8311_codec_new(&codec_cfg);
    if (s_codec_if == NULL) {
        ESP_LOGE(TAG, "es8311_codec_new failed");
        return ESP_FAIL;
    }

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = s_codec_if,
        .data_if = s_data_if,
    };
    s_codec_dev = esp_codec_dev_new(&codec_dev_cfg);
    if (s_codec_dev == NULL) {
        ESP_LOGE(TAG, "esp_codec_dev_new failed");
        return ESP_FAIL;
    }
    s_play_dev = s_codec_dev;
    s_record_dev = s_codec_dev;

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = AUDIO_SAMPLE_RATE_HZ,
        .channel = AUDIO_CHANNELS,
        .bits_per_sample = AUDIO_BITS_PER_SAMPLE,
        .mclk_multiple = 256,
    };

    ESP_RETURN_ON_ERROR(esp_codec_dev_open(s_codec_dev, &fs), TAG, "open codec device failed");
    ESP_RETURN_ON_ERROR(esp_codec_dev_set_out_vol(s_codec_dev, AUDIO_VOLUME_PERCENT), TAG, "set output volume failed");
    ESP_RETURN_ON_ERROR(esp_codec_dev_set_in_gain(s_codec_dev, AUDIO_IN_GAIN_DB), TAG, "set input gain failed");

    ESP_LOGI(TAG, "ES8311 ready: addr=0x%02X mode=both sample_rate=%d channels=%d bits=%d volume=%d%% gain=%.1f dB",
             ES8311_CODEC_DEFAULT_ADDR, AUDIO_SAMPLE_RATE_HZ, AUDIO_CHANNELS, AUDIO_BITS_PER_SAMPLE,
             AUDIO_VOLUME_PERCENT, AUDIO_IN_GAIN_DB);
    return ESP_OK;
}

esp_err_t uart_audio_init(void)
{
    uart_config_t cfg = {
        .baud_rate = UART_AUDIO_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_param_config(UART_AUDIO_PORT, &cfg), TAG, "uart param config failed");
    ESP_RETURN_ON_ERROR(uart_set_baudrate(UART_AUDIO_PORT, UART_AUDIO_BAUD_RATE), TAG, "uart baudrate config failed");

    uart_vfs_dev_use_nonblocking(UART_AUDIO_PORT);
    uart_vfs_dev_port_set_rx_line_endings(UART_AUDIO_PORT, ESP_LINE_ENDINGS_LF);
    uart_vfs_dev_port_set_tx_line_endings(UART_AUDIO_PORT, ESP_LINE_ENDINGS_LF);

    int stdin_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (stdin_flags < 0) {
        ESP_LOGE(TAG, "get stdin flags failed errno=%d", errno);
        return ESP_FAIL;
    }
    if (fcntl(STDIN_FILENO, F_SETFL, stdin_flags | O_NONBLOCK) < 0) {
        ESP_LOGE(TAG, "set stdin nonblocking failed errno=%d", errno);
        return ESP_FAIL;
    }

    uint8_t bitbucket[32];
    while (read(STDIN_FILENO, bitbucket, sizeof(bitbucket)) > 0) {
    }

    ESP_LOGI(TAG, "UART ready: port=%d baud=%d", UART_AUDIO_PORT, UART_AUDIO_BAUD_RATE);
    return ESP_OK;
}

esp_err_t codec_request_capture(int duration_sec)
{
    if (duration_sec < GET_AUDIO_MIN_SECONDS || duration_sec > GET_AUDIO_MAX_SECONDS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!capture_try_acquire()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueSend(s_capture_cmd_queue, &duration_sec, 0) != pdTRUE) {
        capture_release();
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void get_audio(int duration_sec)
{
    const size_t total_bytes = (size_t)duration_sec * AUDIO_SAMPLE_RATE_HZ * AUDIO_CHANNELS * (AUDIO_BITS_PER_SAMPLE / 8);
    capture_stream_info_t info = {
        .duration_sec = duration_sec,
        .total_bytes = total_bytes,
    };
    int16_t record_pcm[AUDIO_SAMPLES_ONECH_CHUNK * AUDIO_CHANNELS];
    int16_t out_pcm[AUDIO_SAMPLES_ONECH_CHUNK * AUDIO_CHANNELS];
    size_t bytes_remaining = total_bytes;

    xQueueSend(s_uart_stream_queue, &info, portMAX_DELAY);

    while (bytes_remaining > 0U) {
        size_t bytes_this_chunk = (bytes_remaining > AUDIO_BYTES_PER_CHUNK) ? AUDIO_BYTES_PER_CHUNK : bytes_remaining;
        size_t frames_this_chunk = bytes_this_chunk / AUDIO_BYTES_PER_SAMPLE;

        if (esp_codec_dev_read(s_record_dev, record_pcm, (int)bytes_this_chunk) != ESP_CODEC_DEV_OK) {
            memset(record_pcm, 0, bytes_this_chunk);
        }

        prepare_stereo_output(record_pcm, out_pcm, frames_this_chunk);
        // (void)esp_codec_dev_write(s_play_dev, out_pcm, (int)bytes_this_chunk);

        while (xRingbufferSend(s_uart_audio_rb, out_pcm, bytes_this_chunk, pdMS_TO_TICKS(100)) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        bytes_remaining -= bytes_this_chunk;
    }

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

void uart_audio_tx_task(void *arg)
{
    (void)arg;
    capture_stream_info_t info;

    while (true) {
        if (xQueueReceive(s_uart_stream_queue, &info, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        xSemaphoreTake(s_uart_tx_mutex, portMAX_DELAY);
        uart_printf_unlocked("AUDIO_BEGIN duration_s=%d sample_rate=%d channels=%d bits=%d total_bytes=%lu\n",
                             info.duration_sec, AUDIO_SAMPLE_RATE_HZ, AUDIO_CHANNELS, AUDIO_BITS_PER_SAMPLE,
                             (unsigned long)info.total_bytes);

        size_t bytes_remaining = info.total_bytes;
        while (bytes_remaining > 0U) {
            size_t item_size = 0;
            uint8_t *item = (uint8_t *)xRingbufferReceiveUpTo(s_uart_audio_rb, &item_size, pdMS_TO_TICKS(1000),
                                                              bytes_remaining);
            if (item == NULL) {
                continue;
            }

            uart_write_all_unlocked(item, item_size);
            bytes_remaining -= item_size;
            vRingbufferReturnItem(s_uart_audio_rb, item);
        }

        uart_printf_unlocked("\nAUDIO_END total_bytes=%lu\n", (unsigned long)info.total_bytes);
        uart_wait_output_idle();
        xSemaphoreGive(s_uart_tx_mutex);

        capture_release();
        xTaskNotifyGive(s_capture_task_handle);
    }
}

void audio_capture_task(void *arg)
{
    (void)arg;
    int duration_sec = 0;

    while (true) {
        if (xQueueReceive(s_capture_cmd_queue, &duration_sec, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        get_audio(duration_sec);
    }
}

void uart_cmd_task(void *arg)
{
    (void)arg;
    char cmd[UART_CMD_MAX_LEN];
    size_t cmd_len = 0;
    TickType_t last_ready_tick = 0;

    while (true) {
        TickType_t now = xTaskGetTickCount();
        if (!codec_capture_is_busy() && cmd_len == 0U &&
            (last_ready_tick == 0 || (now - last_ready_tick) >= pdMS_TO_TICKS(500))) {
            uart_advertise_ready();
            last_ready_tick = now;
        }

        uint8_t ch = 0;
        ssize_t read_len = read(STDIN_FILENO, &ch, 1);
        if (read_len < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGW(TAG, "stdin read failed errno=%d", errno);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (read_len == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            cmd[cmd_len] = '\0';
            if (cmd_len > 0U) 
            {
                int duration_sec = 0;
                if (parse_feed_command(cmd)) 
                {
                    servo_feedMiao_once(AUTO_FEED_TIME_S);
                    uart_printf_locked("FEED_OK\n");

                }
                else if (parse_get_audio_command(cmd, &duration_sec)) 
                {
                    esp_err_t err = codec_request_capture(duration_sec);
                    if (err == ESP_ERR_TIMEOUT) {
                        uart_printf_locked("AUDIO_ERROR reason=queue_full requested=%d max_seconds=%d\n",
                                           duration_sec, GET_AUDIO_MAX_SECONDS);
                    } else if (err == ESP_ERR_INVALID_STATE) {
                        uart_printf_locked("AUDIO_ERROR reason=busy requested=%d max_seconds=%d\n",
                                           duration_sec, GET_AUDIO_MAX_SECONDS);
                    }

                }
                 else if (!codec_capture_is_busy()) 
                {
                    uart_printf_locked("AUDIO_ERROR reason=unsupported_command requested=0 max_seconds=%d\n",
                                       GET_AUDIO_MAX_SECONDS);

                }
            }
            cmd_len = 0;
            continue;
        }

        if (cmd_len < sizeof(cmd) - 1U) {
            cmd[cmd_len++] = (char)ch;
        } else {
            cmd_len = 0;
            if (!codec_capture_is_busy()) {
                uart_printf_locked("AUDIO_ERROR reason=command_too_long requested=0 max_seconds=%d\n",
                                   GET_AUDIO_MAX_SECONDS);
            }
        }
    }
}

esp_err_t audio_storage_init(void)
{
    static bool mounted = false;

    if (mounted) {
        return ESP_OK;
    }

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "audio",
        .max_files = 6,
        .format_if_mount_failed = false,
    };

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
    mounted = true;
    return ESP_OK;
}

bool audio_play_owner_voice(void)
{
    FILE *fp = NULL;
    bool ok = false;

    int16_t pcm_buf[AUDIO_SAMPLES_ONECH_CHUNK * AUDIO_CHANNELS];

    if (!capture_try_acquire()) {
        ESP_LOGW(TAG, "audio busy, skip owner voice play");
        return false;
    }

    fp = fopen(OWNER_VOICE_FILE, "rb");
    if (fp == NULL) {
        ESP_LOGW(TAG, "owner voice file not found: %s", OWNER_VOICE_FILE);
        goto exit;
    }

    ESP_LOGI(TAG, "start owner voice play: %s", OWNER_VOICE_FILE);

    while (1) {
        size_t read_bytes = fread(pcm_buf, 1, sizeof(pcm_buf), fp);
        if (read_bytes == 0U) {
            break;
        }

        if (esp_codec_dev_write(s_play_dev, pcm_buf, (int)read_bytes) != ESP_CODEC_DEV_OK) {
            ESP_LOGE(TAG, "play owner voice failed");
            goto exit;
        }
    }

    ok = true;
    ESP_LOGI(TAG, "owner voice play done");

exit:
    if (fp != NULL) {
        fclose(fp);
    }
    capture_release();
    return ok;
}


bool audio_record_owner_voice(void)
{
    FILE *fp = NULL;
    bool ok = false;

    const size_t total_bytes =
        (size_t)RECORD_DURATION_SEC * AUDIO_SAMPLE_RATE_HZ * AUDIO_CHANNELS * (AUDIO_BITS_PER_SAMPLE / 8);

    int16_t pcm_buf[AUDIO_SAMPLES_ONECH_CHUNK * AUDIO_CHANNELS];
    size_t bytes_remaining = total_bytes;

    if (!capture_try_acquire()) {
        ESP_LOGW(TAG, "audio busy, skip owner voice record");
        return false;
    }

    fp = fopen(OWNER_VOICE_FILE, "wb");
    if (fp == NULL) {
        ESP_LOGE(TAG, "open record file failed: %s", OWNER_VOICE_FILE);
        goto exit;
    }

    ESP_LOGI(TAG, "start owner voice record: %d s -> %s", RECORD_DURATION_SEC, OWNER_VOICE_FILE);

    while (bytes_remaining > 0U) {
        size_t bytes_this_chunk =
            (bytes_remaining > AUDIO_BYTES_PER_CHUNK) ? AUDIO_BYTES_PER_CHUNK : bytes_remaining;

        if (esp_codec_dev_read(s_record_dev, pcm_buf, (int)bytes_this_chunk) != ESP_CODEC_DEV_OK) {
            memset(pcm_buf, 0, bytes_this_chunk);
        }

        if (fwrite(pcm_buf, 1, bytes_this_chunk, fp) != bytes_this_chunk) {
            ESP_LOGE(TAG, "write record file failed");
            goto exit;
        }

        bytes_remaining -= bytes_this_chunk;
    }

    fflush(fp);
    ok = true;
    ESP_LOGI(TAG, "owner voice record done");

exit:
    if (fp != NULL) {
        fclose(fp);
    }
    capture_release();
    return ok;
}
