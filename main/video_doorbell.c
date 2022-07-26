/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Agora Lab, Inc (http://www.agora.io/)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <string.h>

#include "app_config.h"
#include "agora_iot_api.h"
#include "agora_iot_call.h"
#include "agora_iot_device_manager.h"

#include "doorbell_dp_common.h"
#include "device_state.h"

#include "algorithm_stream.h"
#include "audio_element.h"
#include "audio_hal.h"
#include "audio_mem.h"
#include "audio_pipeline.h"
#include "audio_sys.h"
#include "es7210.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_jpeg_enc.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_wifi.h"
#include "filter_resample.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "i2c_bus.h"
#include "i2s_stream.h"
#include "input_key_service.h"
#include "nvs_flash.h"
#include "raw_stream.h"
#include "quirc.h"
#include "convert.h"

#include "cJSON.h"

static const char *TAG = "Agora";

#define DEFAULT_LISTEN_INTERVAL CONFIG_EXAMPLE_WIFI_LISTEN_INTERVAL

#if CONFIG_EXAMPLE_POWER_SAVE_MIN_MODEM
#define DEFAULT_PS_MODE WIFI_PS_MIN_MODEM
#elif CONFIG_EXAMPLE_POWER_SAVE_MAX_MODEM
#define DEFAULT_PS_MODE WIFI_PS_MAX_MODEM
#elif CONFIG_EXAMPLE_POWER_SAVE_NONE
#define DEFAULT_PS_MODE WIFI_PS_NONE
#else
#define DEFAULT_PS_MODE WIFI_PS_NONE
#endif

#define CAMERA_WIDTH (CONFIG_FRAME_WIDTH)
#define CAMERA_HIGH (CONFIG_FRAME_HIGH)

#define CAM_PIN_PWDN -1 // power down is not used
#define CAM_PIN_RESET -1 // software reset will be performed
#define CAM_PIN_XCLK GPIO_NUM_40
#define CAM_PIN_SIOD GPIO_NUM_17
#define CAM_PIN_SIOC GPIO_NUM_18

#define CAM_PIN_D7 GPIO_NUM_39
#define CAM_PIN_D6 GPIO_NUM_41
#define CAM_PIN_D5 GPIO_NUM_42
#define CAM_PIN_D4 GPIO_NUM_12
#define CAM_PIN_D3 GPIO_NUM_3
#define CAM_PIN_D2 GPIO_NUM_14
#define CAM_PIN_D1 GPIO_NUM_47
#define CAM_PIN_D0 GPIO_NUM_13
#define CAM_PIN_VSYNC GPIO_NUM_21
#define CAM_PIN_HREF GPIO_NUM_38
#define CAM_PIN_PCLK GPIO_NUM_11

#define PRIO_TASK_FETCH (21)

#define DEFAULT_PCM_CAPTURE_LEN (2048)

#ifdef CONFIG_AUDIO_SAMPLE_RATE_8K
#define I2S_SAMPLE_RATE 8000
#else
#define I2S_SAMPLE_RATE 16000
#endif
#define I2S_CHANNELS 1
#define I2S_BITS 16

#define ESP_READ_BUFFER_SIZE 1024

#define DEFAULT_MAX_BITRATE (2000000)

typedef struct {
  bool b_wifi_connected;
  bool b_call_session_started;
  bool b_exit;
  sys_up_mode_e up_mode;
} app_t;

static app_t g_app = {
    .b_call_session_started = false,
    .b_wifi_connected       = false,
    .b_exit                 = false,
    .up_mode                = SYS_UP_MODE_POWERON,
};

static camera_config_t camera_config = {
  .pin_pwdn  = CAM_PIN_PWDN,
  .pin_reset = CAM_PIN_RESET,
  .pin_xclk  = CAM_PIN_XCLK,
  .pin_sscb_sda = CAM_PIN_SIOD,
  .pin_sscb_scl = CAM_PIN_SIOC,

  .pin_d7 = CAM_PIN_D7,
  .pin_d6 = CAM_PIN_D6,
  .pin_d5 = CAM_PIN_D5,
  .pin_d4 = CAM_PIN_D4,
  .pin_d3 = CAM_PIN_D3,
  .pin_d2 = CAM_PIN_D2,
  .pin_d1 = CAM_PIN_D1,
  .pin_d0 = CAM_PIN_D0,
  .pin_vsync = CAM_PIN_VSYNC,
  .pin_href  = CAM_PIN_HREF,
  .pin_pclk  = CAM_PIN_PCLK,

  // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
  .xclk_freq_hz = 20000000,
  .ledc_timer   = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_YUV422, // YUV422,GRAYSCALE,RGB565,JPEG
  .frame_size   = CONFIG_FRAME_SIZE, // QQVGA-UXGA Do not use sizes above QVGA
  // when not JPEG

  .jpeg_quality = 12, // 0-63 lower number means higher quality
  .fb_count     = 2, // if more than one, i2s runs in continuous mode. Use only with JPEG
  .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
  // .conv_mode    = YUV422_TO_YUV420,
};

static agora_iot_handle_t g_handle = NULL;
static device_handle_t dev_state = NULL;


static uint32_t image_cnt = 0;
static uint32_t tick_begin = 0;
static esp_timer_handle_t fps_timer = NULL;

static void *jpg_encoder;
static audio_element_handle_t raw_read, raw_write, element_algo;
static audio_pipeline_handle_t recorder, player;
// static ringbuf_handle_t ringbuf_r, ringbuf_w;
static SemaphoreHandle_t g_video_capture_sem  = NULL;
static SemaphoreHandle_t g_audio_capture_sem  = NULL;

static QueueHandle_t    g_qr_queue;


static esp_err_t es7210_write_reg(i2c_bus_handle_t i2c_handle, uint8_t reg_addr, uint8_t data)
{
  return i2c_bus_write_bytes(i2c_handle, ES7210_AD1_AD0_00, &reg_addr, sizeof(reg_addr), &data, sizeof(data));
}

static void set_es7210_tdm_mode(void)
{
  i2c_config_t es_i2c_cfg;
  i2c_bus_handle_t i2c_handle = i2c_bus_create(I2C_NUM_0, &es_i2c_cfg);
  es7210_write_reg(i2c_handle, 0x01, 0x20);
  es7210_write_reg(i2c_handle, 0x03, 0x06);
  es7210_write_reg(i2c_handle, 0x04, 0x03);
  es7210_write_reg(i2c_handle, 0x06, 0x04);
  es7210_write_reg(i2c_handle, 0x08, 0x14);
  es7210_write_reg(i2c_handle, 0x0b, 0x01);
  es7210_write_reg(i2c_handle, 0x11, 0x60);
  es7210_write_reg(i2c_handle, 0x12, 0x02);
  es7210_write_reg(i2c_handle, 0x3f, 0x01);
  es7210_write_reg(i2c_handle, 0x43, 0x1e);
  es7210_write_reg(i2c_handle, 0x44, 0x1e);
  es7210_write_reg(i2c_handle, 0x45, 0x18);
  es7210_write_reg(i2c_handle, 0x46, 0x1e);
  es7210_write_reg(i2c_handle, 0x47, 0x08);
  es7210_write_reg(i2c_handle, 0x49, 0x08);
  es7210_write_reg(i2c_handle, 0x4a, 0x08);
}

static void fps_timer_callback(void *arg)
{
  uint32_t cur_tick = xTaskGetTickCount();
  uint32_t duration = cur_tick - tick_begin;
  ESP_LOGW(TAG, "duration %-15u, image cnt %-10u, fps %f", duration, image_cnt,
            (float)image_cnt / (float)(duration / CONFIG_FREERTOS_HZ));

  audio_sys_get_real_time_stats();
  ESP_LOGI(TAG, "MEM Total:%d Bytes, Inter:%d Bytes, Dram:%d Bytes", esp_get_free_heap_size(),
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}

static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
{
  if (evt->type == INPUT_KEY_SERVICE_ACTION_CLICK) {
    switch ((int)evt->data) {
    case INPUT_KEY_USER_ID_REC: {
      char *user_account = NULL;
      if (g_handle == NULL) {
        return ESP_FAIL;
      }
      if (0 != device_get_item_string(dev_state, "bind_user", &user_account)) {
        ESP_LOGE(TAG, "cannot found user_id from device state.\n");
        return ESP_FAIL;
      }

      ESP_LOGW(TAG, "Now ring the bell and call user %s ...\n", user_account);
      agora_iot_call(g_handle, user_account, "I'm the guest");
      if (user_account) {
        free(user_account);
      }
    } break;
    case INPUT_KEY_USER_ID_VOLUP:
      if (g_handle == NULL) {
        return ESP_FAIL;
      }

      ESP_LOGW(TAG, "Hang up the call.");
      agora_iot_hang_up(g_handle);
      g_app.b_call_session_started = false;
      break;
    case INPUT_KEY_USER_ID_VOLDOWN:
      ESP_LOGW(TAG, "Now exiting the app...");
      g_app.b_exit = true;
      break;
    case INPUT_KEY_USER_ID_SET:{
      esp_err_t ret = nvs_flash_erase_partition("nvs");
      ESP_LOGW(TAG, "Erase the nvs flash %d...", ret);
    } break;
    default:
      ESP_LOGE(TAG, "User Key ID[%d] does not support", (int)evt->data);
      break;
    }
  }

  return ESP_OK;
}

static void start_key_service(void)
{
  ESP_LOGI(TAG, "Initialize peripherals");
  esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
  esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

  ESP_LOGI(TAG, "Initialize Button peripheral with board init");
  audio_board_key_init(set);

  ESP_LOGI(TAG, "Create and start input key service");
  input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
  input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
  input_cfg.handle                 = set;
  input_cfg.based_cfg.task_stack   = 5 * 1024;
  // input_cfg.based_cfg.extern_stack = true;
  periph_service_handle_t input_ser = input_key_service_create(&input_cfg);

  input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
  periph_service_set_callback(input_ser, input_key_service_cb, NULL);
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    g_app.b_wifi_connected = true;
    ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
  }
}

/*init wifi as sta and set power save mode*/
static void setup_wifi(void)
{
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));
}

static esp_err_t i2s_data_divided(int16_t *raw_buff, int len, int16_t *buf_aec)
{
  for (int i = 0; i < len / 4; i++) {
    buf_aec[i << 1] = raw_buff[(i << 1) + 1];
    buf_aec[(i << 1) + 1] = raw_buff[i << 1];
  }
  return ESP_OK;
}

static int i2s_stream_read_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
{
  size_t bytes_read = 0;

  char *buf_tmp = audio_calloc(1, ESP_READ_BUFFER_SIZE);
  AUDIO_MEM_CHECK(TAG, buf, return 0);
  char *buf_aec = audio_calloc(1, ESP_READ_BUFFER_SIZE);
  AUDIO_MEM_CHECK(TAG, buf_aec, return 0);

  i2s_read(0, buf_tmp, ESP_READ_BUFFER_SIZE, &bytes_read, wait_time);
  if (bytes_read == ESP_READ_BUFFER_SIZE) {
    i2s_data_divided((int16_t *)buf_tmp, ESP_READ_BUFFER_SIZE, (int16_t *)buf_aec);
    memcpy(buf, buf_aec, bytes_read);
  }

  free(buf_tmp);
  free(buf_aec);

  return bytes_read;
}

static esp_err_t recorder_pipeline_open(void)
{
  audio_element_handle_t i2s_stream_reader;
  audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
  recorder = audio_pipeline_init(&pipeline_cfg);
  AUDIO_NULL_CHECK(TAG, recorder, return ESP_FAIL);

  i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
  i2s_cfg.type          = AUDIO_STREAM_READER;
  i2s_cfg.uninstall_drv = false;
#ifdef CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
  i2s_cfg.i2s_port      = 1;
#endif
  i2s_cfg.task_core     = 1;
  i2s_cfg.i2s_config.channel_format  = I2S_CHANNEL_FMT_ONLY_LEFT;
  i2s_cfg.i2s_config.sample_rate     = I2S_SAMPLE_RATE;
  i2s_cfg.i2s_config.mode            = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
#ifdef CONFIG_ESP32_S3_KORVO2_V3_BOARD
  i2s_cfg.i2s_config.bits_per_sample = 32;
#endif
  i2s_cfg.out_rb_size  = 2 * 1024;
  i2s_cfg.stack_in_ext = true;
  i2s_stream_reader = i2s_stream_init(&i2s_cfg);
#ifdef CONFIG_ESP32_S3_KORVO2_V3_BOARD
  audio_element_set_read_cb(i2s_stream_reader, i2s_stream_read_cb, NULL);
#endif

  algorithm_stream_cfg_t algo_config = ALGORITHM_STREAM_CFG_DEFAULT();
#if defined(CONFIG_ESP_LYRAT_MINI_V1_1_BOARD) || defined(CONFIG_ESP32_S3_KORVO2_V3_BOARD)
  algo_config.input_type = ALGORITHM_STREAM_INPUT_TYPE1;
#else
  algo_config.input_type = ALGORITHM_STREAM_INPUT_TYPE2;
#endif
  // algo_config.task_core = 1;
  algo_config.task_stack = 4 * 1024;
  algo_config.algo_mask  = ALGORITHM_STREAM_USE_AEC;

  element_algo = algo_stream_init(&algo_config);
  audio_element_set_music_info(element_algo, I2S_SAMPLE_RATE, 1, I2S_BITS);

  audio_pipeline_register(recorder, i2s_stream_reader, "i2s");
  audio_pipeline_register(recorder, element_algo, "algo");

  raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
  raw_cfg.type        = AUDIO_STREAM_READER;
  raw_cfg.out_rb_size = 2 * 1024;
  raw_read = raw_stream_init(&raw_cfg);
  audio_element_set_output_timeout(raw_read, portMAX_DELAY);

  audio_pipeline_register(recorder, raw_read, "raw");

  const char *link_tag[3] = { "i2s", "algo", "raw" };
  audio_pipeline_link(recorder, &link_tag[0], 3);

  ESP_LOGI(TAG, "audio recorder has been created");
  return ESP_OK;
}

static esp_err_t player_pipeline_open()
{
  audio_element_handle_t i2s_stream_writer;
  audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
  player = audio_pipeline_init(&pipeline_cfg);
  AUDIO_NULL_CHECK(TAG, player, return ESP_FAIL);

  raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
  raw_cfg.type        = AUDIO_STREAM_WRITER;
  raw_cfg.out_rb_size = 8 * 1024;
  raw_write = raw_stream_init(&raw_cfg);

  i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
  i2s_cfg.type = AUDIO_STREAM_WRITER;
  i2s_cfg.uninstall_drv              = false;
  i2s_cfg.i2s_config.channel_format  = I2S_CHANNEL_FMT_ONLY_LEFT;
  i2s_cfg.i2s_config.mode            = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
  i2s_cfg.i2s_config.sample_rate     = I2S_SAMPLE_RATE;
#ifdef CONFIG_ESP32_S3_KORVO2_V3_BOARD
  i2s_cfg.i2s_config.bits_per_sample = 32;
  i2s_cfg.need_expand                = true;
#endif
#if !defined(CONFIG_ESP_LYRAT_MINI_V1_1_BOARD) && !defined(CONFIG_ESP32_S3_KORVO2_V3_BOARD)
  i2s_cfg.multi_out_num              = 1;
#endif
  i2s_cfg.task_core                  = 1;
  i2s_cfg.stack_in_ext               = true;
  i2s_stream_writer = i2s_stream_init(&i2s_cfg);

  audio_pipeline_register(player, raw_write, "raw");
  audio_pipeline_register(player, i2s_stream_writer, "i2s");
  const char *link_tag[3] = { "raw", "i2s" };
  audio_pipeline_link(player, &link_tag[0], 2);

  return ESP_OK;
}

static void setup_audio(void)
{
  audio_board_handle_t board_handle = audio_board_init();
  audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
  audio_hal_set_volume(board_handle->audio_hal, 75);

  es7210_mic_select(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2 | ES7210_INPUT_MIC3 | ES7210_INPUT_MIC4);
  set_es7210_tdm_mode();

  recorder_pipeline_open();
  player_pipeline_open();
}

static void init_camera(void)
{
  // initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera Init Failed");
  }
}

static void *init_jpeg_encoder(int quality, int hfm_core, int hfm_priority, jpeg_subsampling_t subsampling)
{
  jpeg_enc_info_t jpeg_enc_info   = { 0 };
  jpeg_enc_info.width             = CAMERA_WIDTH;
  jpeg_enc_info.height            = CAMERA_HIGH;
  jpeg_enc_info.src_type          = JPEG_RAW_TYPE_YCbYCr;
  jpeg_enc_info.subsampling       = subsampling;
  jpeg_enc_info.quality           = quality;
  // jpeg_enc_info.task_enable     = true;
  jpeg_enc_info.hfm_task_core     = hfm_core;
  jpeg_enc_info.hfm_task_priority = hfm_priority;
  return jpeg_enc_open(&jpeg_enc_info);
}


static int send_video_frame(uint8_t *data, uint32_t len)
{
    int rval = -1;

    // API: send video data
    ago_video_frame_t ago_frame = { 0 };
    ago_frame.data_type         = AGO_VIDEO_DATA_TYPE_JPEG;
    ago_frame.is_key_frame      = true;
    ago_frame.video_buffer      = data;
    ago_frame.video_buffer_size = len;
    rval = agora_iot_push_video_frame(g_handle, &ago_frame);
    if (rval < 0) {
        ESP_LOGE(TAG, "Failed to push video frame");
        return -1;
    }

    return 0;
}

static int send_audio_frame(uint8_t *data, uint32_t len)
{
    int rval = -1;

    // API: send audio data
    ago_audio_frame_t ago_frame = { 0 };
    ago_frame.data_type         = AGO_AUDIO_DATA_TYPE_PCM;
    ago_frame.audio_buffer      = data;
    ago_frame.audio_buffer_size = len;
    rval = agora_iot_push_audio_frame(g_handle, &ago_frame);
    if (rval < 0) {
        ESP_LOGE(TAG, "Failed to push audio frame");
        return -1;
    }

    return 0;
}

static void video_capture_and_send_task(void *args)
{
  const int image_buf_len = 30 * 1024;
  int image_len = 0;

#ifdef CONFIG_ENABLE_RUN_TIME_STATS
  esp_timer_create_args_t create_args = { .callback = fps_timer_callback, .arg = NULL, .name = "fps timer" };
  esp_timer_create(&create_args, &fps_timer);
  esp_timer_start_periodic(fps_timer, 20 * 1000 * 1000);
  tick_begin = xTaskGetTickCount();
  image_cnt = 0;
#endif

  uint8_t *image_buf = heap_caps_malloc(image_buf_len, MALLOC_CAP_SPIRAM);
  if (!image_buf) {
    ESP_LOGE(TAG, "Failed to alloc video buffer!");
    return;
  }

  while (1) {
    xSemaphoreTake(g_video_capture_sem, portMAX_DELAY);

    while (g_app.b_call_session_started) {
      camera_fb_t *pic = esp_camera_fb_get();
      image_cnt++;

      jpeg_enc_process(jpg_encoder, pic->buf, pic->len, image_buf, image_buf_len, &image_len);

      send_video_frame(image_buf, image_len);

      esp_camera_fb_return(pic);
    }
  }
  free(image_buf);
  jpeg_enc_close(jpg_encoder);

#ifdef CONFIG_ENABLE_RUN_TIME_STATS
  esp_timer_stop(fps_timer);
  esp_timer_delete(fps_timer);
#endif
  vTaskDelete(NULL);
}

static void audio_capture_and_send_task(void *threadid)
{
  int read_len = DEFAULT_PCM_CAPTURE_LEN;
  int ret;

  uint8_t *pcm_buf = heap_caps_malloc(read_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!pcm_buf) {
    ESP_LOGE(TAG, "Failed to alloc audio buffer!");
    return;
  }

  audio_pipeline_run(recorder);
  audio_pipeline_run(player);

  while (1) {
    xSemaphoreTake(g_audio_capture_sem, portMAX_DELAY);

    while (g_app.b_call_session_started) {
      ret = raw_stream_read(raw_read, (char *)pcm_buf, read_len);
      if (ret != read_len) {
        ESP_LOGW(TAG, "write error, expect %d, but only %d", read_len, ret);
      }
      send_audio_frame(pcm_buf, DEFAULT_PCM_CAPTURE_LEN);
    }
  }

  audio_pipeline_terminate(recorder);
  audio_pipeline_terminate(player);

  free(pcm_buf);
  vTaskDelete(NULL);
}

static void create_capture_task(void)
{
  int rval;

#ifndef CONFIG_AUDIO_ONLY
  g_video_capture_sem = xSemaphoreCreateBinary();
  if (NULL == g_video_capture_sem) {
    ESP_LOGE(TAG, "Unable to create video capture semaphore!");
    return;
  }
  rval = xTaskCreatePinnedToCore(video_capture_and_send_task, "video_task", 3 * 1024, NULL, PRIO_TASK_FETCH, NULL, 1);
  if (rval != pdTRUE) {
    ESP_LOGE(TAG, "Unable to create video capture thread!");
    return;
  }
#endif

  g_audio_capture_sem = xSemaphoreCreateBinary();
  if (NULL == g_audio_capture_sem) {
    ESP_LOGE(TAG, "Unable to create audio capture semaphore!");
    return;
  }
  rval = xTaskCreatePinnedToCore(audio_capture_and_send_task, "audio_task", 3 * 1024, NULL, PRIO_TASK_FETCH, NULL, 1);
  if (rval != pdTRUE) {
    ESP_LOGE(TAG, "Unable to create audio capture thread!");
    return;
  }
}

static void iot_cb_call_request(const char *peer_name, const char *attach_msg)
{
  if (!peer_name) {
    return;
  }

  ESP_LOGI(TAG, "Get call from peer \"%s\", attach message: %s", peer_name, attach_msg ? attach_msg : "null");

  agora_iot_answer(g_handle);
}

static void iot_cb_start_push_frame(void)
{
  ESP_LOGI(TAG, "Start push audio/video frames");
  g_app.b_call_session_started = true;

#ifndef CONFIG_AUDIO_ONLY
  xSemaphoreGive(g_video_capture_sem);
#endif

  xSemaphoreGive(g_audio_capture_sem);
}

static void iot_cb_stop_push_frame(void)
{
  ESP_LOGI(TAG, "Stop push audio/video frames");
  g_app.b_call_session_started = false;
}

static void iot_cb_call_hung_up(const char *peer_name)
{
  if (!peer_name) {
    ESP_LOGI(TAG, "Get hangup from peer \"%s\"", peer_name);
  }
}

static void iot_cb_call_answered(const char *peer_name)
{
  if (!peer_name) {
    ESP_LOGI(TAG, "Get answer from peer \"%s\"", peer_name);
  }
}

static void iot_cb_call_timeout(const char *peer_name)
{
  if (!peer_name) {
    ESP_LOGI(TAG, "No answer from peer \"%s\"", peer_name);
  }
}

static void iot_cb_receive_video_frame(ago_video_frame_t *frame)
{
}

static void iot_cb_receive_audio_frame(ago_audio_frame_t *frame)
{
  raw_stream_write(raw_write, (char *)frame->audio_buffer, frame->audio_buffer_size);
}


static void qr_recoginze(void *parameter)
{
  // Save image width and height, avoid allocate memory repeatly.
  uint16_t old_width  = 0;
  uint16_t old_height = 0;

  // Construct a new QR-code recognizer.
  ESP_LOGI(TAG, "Construct a new QR-code recognizer(quirc).");
  struct quirc *qr_recognizer = quirc_new();
  if (!qr_recognizer) {
    ESP_LOGE(TAG, "Can't create quirc object");
  }
  camera_fb_t *fb = NULL;
  uint8_t *image  = NULL;
  int id_count    = 0;

  while (1) {
    // Capture a frame
    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        continue;
    }

    if (old_width != fb->width || old_height != fb->height) {
      // Resize the QR-code recognizer.
      if (quirc_resize(qr_recognizer, fb->width, fb->height) < 0) {
        ESP_LOGE(TAG, "Resize the QR-code recognizer err.");
        esp_camera_fb_return(fb);
        continue;
      } else {
        old_width  = fb->width;
        old_height = fb->height;
        ESP_LOGI(TAG, "Resize QR-code: width %d, height %d", old_width, old_height);
      }
    }

    /** These functions are used to process images for QR-code recognition.
     * quirc_begin() must first be called to obtain access to a buffer into
     * which the input image should be placed. Optionally, the current
     * width and height may be returned.
     *
     * After filling the buffer, quirc_end() should be called to process
     * the image for QR-code recognition. The locations and content of each
     * code may be obtained using accessor functions described below.
     */
    image = quirc_begin(qr_recognizer, NULL, NULL);
    // 图像进行灰度化
		yuyv_to_luma(fb->buf, fb->width * 2, fb->width, fb->height, image, fb->width);
    quirc_end(qr_recognizer);

    // Return the number of QR-codes identified in the last processed image.
    id_count = quirc_count(qr_recognizer);
    if (id_count == 0) {
      esp_camera_fb_return(fb);
      ESP_LOGE(TAG, "Error: not a valid qrcode");
      continue;
    }

#ifdef CONFIG_SDCARD
    static int cnt = 0;

    char file_name[64] = { 0 };
    snprintf(file_name, 64, "/sdcard/pic%03d.jpg", cnt++);
    FILE *fp = fopen(file_name, "wb");

    if (NULL == fp) {
      ESP_LOGE(TAG, "%s open fail", file_name);
    } else {
      fwrite(fb->buf, 1, fb->len, fp);
      fclose(fp);
    }
#endif

    esp_camera_fb_return(fb);

    // Print information of QR-code
    struct quirc_code code;
    struct quirc_data qd;
    quirc_extract(qr_recognizer, 0, &code);
    quirc_decode_error_t ret = quirc_decode(&code, &qd);

    if (ret == QUIRC_ERROR_DATA_ECC) {
      // ESP_LOGE(TAG, "quirc_decode err: %s\n", quirc_strerror(ret));

      // 尝试图像水平镜像的quirc解析
			quirc_flip(&code);
			ret = quirc_decode(&code, &qd);
      if (QUIRC_SUCCESS != ret) {
        ESP_LOGE(TAG, "quirc_flip quirc_decode err: %s\n", quirc_strerror(ret));
      }
		}

    if (ret == QUIRC_SUCCESS) {
      //tell
      if (qd.payload_len > 512) {
        ESP_LOGE(TAG, "qd payload len %d\n", qd.payload_len);
      }
      xQueueSend(g_qr_queue, qd.payload, portMAX_DELAY);
      break;
    }
  }

  // Destroy QR-Code recognizer (quirc)
  quirc_destroy(qr_recognizer);
  ESP_LOGI(TAG, "Deconstruct QR-Code recognizer(quirc)");
  vTaskDelete(NULL);
}

static int parse_qrcode_content(device_handle_t dev_state, const char *content)
{
  int ret           = -1;

  cJSON *root = cJSON_Parse(content);
  if (NULL == root) {
    ESP_LOGE(TAG, "cannot parse QRcode: %s\n", content);
    goto qrcode_parse_err;
  }

  // get ssid
  cJSON *item = cJSON_GetObjectItemCaseSensitive(root, "s");
  if (cJSON_GetStringValue(item)) {
    device_set_item_string(dev_state, "ssid", cJSON_GetStringValue(item));
  } else {
    printf("#### cannot found ssid in QRcode !\n");
  }
  item = NULL;

  // get pws
  item = cJSON_GetObjectItemCaseSensitive(root, "p");
  if (cJSON_GetStringValue(item)) {
    device_set_item_string(dev_state, "password", cJSON_GetStringValue(item));
  } else {
    printf("#### cannot found password in QRcode !\n");
  }
  item = NULL;

  // get product key
  item = cJSON_GetObjectItemCaseSensitive(root, "k");
  if (cJSON_GetStringValue(item)) {
    device_set_item_string(dev_state, "product_key", cJSON_GetStringValue(item));
  } else {
    printf("#### cannot found  product key in QRcode !\n");
  }
  item = NULL;

  // get user id
  item = cJSON_GetObjectItemCaseSensitive(root, "u");
  if (cJSON_GetStringValue(item)) {
    device_set_item_string(dev_state, "user_id", cJSON_GetStringValue(item));
  } else {
    printf("#### cannot found user id in QRcode !\n");
  }
  item = NULL;

  // get device name
  item = cJSON_GetObjectItemCaseSensitive(root, "n");
  if (cJSON_GetStringValue(item)) {
    device_set_item_string(dev_state, "device_name", cJSON_GetStringValue(item));
  } else {
    printf("#### cannot found device_name in QRcode, use the default: %s !\n", get_device_id());
    device_set_item_string(dev_state, "device_name", get_device_id());
  }
  item = NULL;
  ret = 0;

qrcode_parse_err:
  if (root) {
    cJSON_Delete(root);
  }
  return ret;
}


static int agora_network_config(device_handle_t dev_state)
{
#define QRCODE_BUF_LEN   512
  char qrcode_content[QRCODE_BUF_LEN + 1] = { 0 };
  ESP_LOGI(TAG, "\n\n------------------ Please input QRcode string with JSON type ----------------------\n");

  g_qr_queue = xQueueCreate(2, QRCODE_BUF_LEN);
  if (NULL == g_qr_queue) {
    ESP_LOGE(TAG, "create semaphore failed !\n");
    return -1;
  }

  // create the task for QR-Code
  xTaskCreate(qr_recoginze, "qr_recoginze_task", 1024 * 40, NULL, 5, NULL);

  // wait for the QR-Code from the qr_reconginze_task
  xQueueReceive(g_qr_queue, qrcode_content, portMAX_DELAY);

  // scanf("%512s", qrcode_content);
  ESP_LOGI(TAG, "qrcode context: %s\n", qrcode_content);
  ESP_LOGI(TAG, "-------------------- Got string and parse it now ------------------------------------\n");

  // raw_stream_write(raw_write, (char *)pcm_test_data, sizeof(pcm_test_data));

  // parse content
  return parse_qrcode_content(dev_state, qrcode_content);
}

static int device_connect_network(char *ssid, char *psw)
{
  wifi_config_t wifi_config = { 0 };

  memcpy((char *)wifi_config.sta.ssid, (char *)ssid, sizeof(wifi_config.sta.ssid));
  memcpy((char *)wifi_config.sta.password, (char *)psw, sizeof(wifi_config.sta.password));
  wifi_config.sta.listen_interval = DEFAULT_LISTEN_INTERVAL;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "esp_wifi_set_ps().");
#ifdef CONFIG_ENABLE_LOW_POWER_MODE
  esp_wifi_set_ps(DEFAULT_PS_MODE);
#else
  esp_wifi_set_ps(WIFI_PS_NONE);
#endif

  return 0;
}

static device_handle_t agora_device_bringup(sys_up_mode_e mode)
{
  char *ssid    = NULL;
  char *psw     = NULL;
  char *license = NULL;
  char *product_key = NULL;

  // 1. load device state config
  device_handle_t dev_state = load_devic_state();

  // 2. do network config process
  if (NULL == dev_state) {
    if (SYS_UP_MODE_WAKEUP == mode) {
      ESP_LOGE(TAG, "device state config must be here is that system was wakeup from low-power mode");
      goto dev_bringup_err;
    }

    dev_state = device_create_state();
    if (NULL == dev_state) {
      ESP_LOGE(TAG, "cannot create device state items !\n");
      goto dev_bringup_err;
    }

    if (0 != agora_network_config(dev_state)) {
      ESP_LOGE(TAG, "config network failed!\n");
      goto dev_bringup_err;
    }
    ESP_LOGI(TAG, "agora_network_config ok\n"); 
  } else {
    ESP_LOGI(TAG, "device load ok\n");
  }

  // 3. connect network (TODO: maybe need not do it on some system)
  if (0 != device_get_item_string(dev_state, "ssid", &ssid) ||
      0 != device_get_item_string(dev_state, "password", &psw)) {
    ESP_LOGE(TAG, "cannot found ssid and password !\n");
    goto dev_bringup_err;
  }
  if (0 != device_connect_network(ssid, psw)) {
    ESP_LOGE(TAG, "cannot connect network !\n");
    goto dev_bringup_err;
  }

  // Wait until WiFi is connected
  while (!g_app.b_wifi_connected) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  if (0 != device_get_item_string(dev_state, "product_key", &product_key)) {
    ESP_LOGE(TAG, "cannot found product_key in device state items\n");
    goto dev_bringup_err;
  }

  // 4. activate the device
  char user_account[64] = { 0 };
  if (SYS_UP_MODE_WAKEUP != mode) {
    // maybe nee to avtiate device if was not wakeup frome low-power mode
    if (SYS_UP_MODE_RESTORE == mode || 0 != agora_iot_query_user(CONFIG_MASTER_SERVER_URL, product_key, get_device_id(), user_account) ||
        0 == strlen(user_account) || 0 != device_get_item_string(dev_state, "license", &license)) {
      // active device if cannot found license info or cannot found bind user info
      if (0 != activate_device(dev_state)) {
        ESP_LOGE(TAG, "cannot activate device !\n");
        goto dev_bringup_err;
      }

      // save state items to file or flash
      save_device_state(dev_state);
    }
  }

  if (license) {
    free(license);
  }
  return dev_state;

dev_bringup_err:
  if (dev_state) {
    device_destroy_state(dev_state);
  }

  if (license) {
    free(license);
  }
  return NULL;
}

static agora_iot_handle_t connect_agora_iot_service(device_handle_t dev_state)
{
  agora_iot_handle_t handle = NULL;
  char *domain = NULL;
  char *dev_crt = NULL;
  char *dev_key = NULL;
  char *license = NULL;
  char *product_key = NULL;
  char *client_id = NULL;

  if (0 != device_get_item_string(dev_state, "dev_crt", &dev_crt) ||
      0 != device_get_item_string(dev_state, "dev_key", &dev_key) ||
      0 != device_get_item_string(dev_state, "domain", &domain)   ||
      0 != device_get_item_string(dev_state, "client_id", &client_id)) {
    ESP_LOGE(TAG, "cannot found dev_crt or dev_key or domain in device state items\n");
    goto agora_iot_err;
  }

  if (0 != device_get_item_string(dev_state, "license", &license)) {
    ESP_LOGE(TAG, "cannot found license in device state items\n");
    goto agora_iot_err;
  }

  if (0 != device_get_item_string(dev_state, "product_key", &product_key)) {
    ESP_LOGE(TAG, "cannot found product_key in device state items\n");
    goto agora_iot_err;
  }

  agora_iot_config_t cfg = {
    .app_id      = CONFIG_AGORA_APP_ID,
    .product_key = product_key,
    .client_id   = client_id,
    .domain      = domain,
    .root_ca     = CONFIG_AWS_ROOT_CA,
    .client_crt  = dev_crt,
    .client_key  = dev_key,
    .enable_rtc  = true,
    .certificate = license,
    .enable_recv_audio = true,
    .enable_recv_video = false,
    .rtc_cb = {
      .cb_start_push_frame    = iot_cb_start_push_frame,
      .cb_stop_push_frame     = iot_cb_stop_push_frame,
      .cb_receive_audio_frame = iot_cb_receive_audio_frame,
      .cb_receive_video_frame = iot_cb_receive_video_frame,
  #ifdef CONFIG_SEND_H264_FRAMES
      .cb_target_bitrate_changed = iot_cb_target_bitrate_changed,
      .cb_key_frame_requested    = iot_cb_key_frame_requested,
  #endif
    },
    .disable_rtc_log      = true,
    .max_possible_bitrate = DEFAULT_MAX_BITRATE,
    .enable_audio_config  = true,
    .audio_config = {
#ifdef CONFIG_AUDIO_SAMPLE_RATE_8K
      .audio_codec_type = AGO_AUDIO_CODEC_TYPE_G711U,
#else
      .audio_codec_type = AGO_AUDIO_CODEC_TYPE_G722,
#endif
#if defined(CONFIG_SEND_PCM_DATA)
      .pcm_sample_rate  = I2S_SAMPLE_RATE,
      .pcm_channel_num  = I2S_CHANNELS,
#endif
    },

    .slave_server_url = CONFIG_SLAVE_SERVER_URL,
    .call_cb = {
      .cb_call_request       = iot_cb_call_request,
      .cb_call_answered      = iot_cb_call_answered,
      .cb_call_hung_up       = iot_cb_call_hung_up,
      .cb_call_local_timeout = iot_cb_call_timeout,
      .cb_call_peer_timeout  = iot_cb_call_timeout,
    },
  };
  handle = agora_iot_init(&cfg);
  if (NULL == handle) {
    ESP_LOGE(TAG, "agora_iot_init failed\n");
    goto agora_iot_err;
  }

agora_iot_err:
  if (domain) {
    free(domain);
  }
  if (dev_crt) {
    free(dev_crt);
  }
  if (dev_key) {
    free(dev_key);
  }
  if (license) {
    free(license);
  }
  return handle;
}

int app_main(void)
{
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

#ifdef CONFIG_ENABLE_LOW_POWER_MODE
  esp_pm_config_esp32s3_t pm_config = { .max_freq_mhz = CONFIG_EXAMPLE_MAX_CPU_FREQ_MHZ,
                                        .min_freq_mhz = CONFIG_EXAMPLE_MIN_CPU_FREQ_MHZ,
                                        .light_sleep_enable = true };
  ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif
  setup_wifi();
  setup_audio();

  init_camera();

  jpg_encoder = init_jpeg_encoder(40, 0, 20, JPEG_SUB_SAMPLE_YUV420);
  if (!jpg_encoder) {
    ESP_LOGE(TAG, "Failed to initialize jpeg encoder!");
    goto EXIT;
  }

  create_capture_task();

#ifdef CONFIG_SDCARD
    ESP_LOGI(TAG, "[1.0] Mount sdcard");
    // Initialize peripherals management
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    // Initialize SD Card peripheral
    audio_board_sdcard_init(set, SD_MODE_1_LINE);
#endif

  // Monitor the key event
  start_key_service();

  g_app.up_mode = SYS_UP_MODE_POWERON;

  ESP_LOGI(TAG, "step1: start init\n");

  // Initialize Agora IoT SDK
  dev_state = agora_device_bringup(g_app.up_mode);
  if (NULL == dev_state) {
    ESP_LOGE(TAG, "connect_agora_iot_service failed.\n");
    goto EXIT;   
  }
  ESP_LOGI(TAG, "step2: device bringup ok\n");

  // connect to agora iot service
  g_handle = connect_agora_iot_service(dev_state);
  if (NULL == g_handle) {
    ESP_LOGE(TAG, "connect_agora_iot_service failed.\n");
    goto EXIT;
  }

  // update device state
  if (0 != update_device_work_state(g_handle, g_app.up_mode)) {
    ESP_LOGE(TAG, "agora_iot_init failed\n");
    goto EXIT;
  }

  // Infinite loop
  while (!g_app.b_exit) {
    if (!g_app.b_call_session_started) {
      ESP_LOGW(TAG, "Please press [REC] key to ring the doorbell ...");
    } else {
      ESP_LOGW(TAG, "Now we're in the call. Please press [VOL+] key to hang up ...");
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }

EXIT:
  // Deinit Agora IoT SDK
  if (g_handle) { 
    agora_iot_deinit(g_handle);
  }

  if (dev_state) {
    device_destroy_state(dev_state);
  }

  ESP_LOGW(TAG, "App exited.");
  return 0;
}
