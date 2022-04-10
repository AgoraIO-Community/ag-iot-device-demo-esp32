/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Agora Lab, Inc (http://www.agora.io/)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <string.h>

#include "app_config.h"
#include "agora_iot_api.h"
#include "agora_iot_call.h"
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
#include "i2c_bus.h"
#include "i2s_stream.h"
#include "input_key_service.h"
#include "nvs_flash.h"
#include "raw_stream.h"

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

#define CAM_PIN_PWDN -1   // power down is not used
#define CAM_PIN_RESET -1  // software reset will be performed
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
#define RING_BUFFER_SIZE (8192)

#define BASIC_AUTH                                                             \
  "ODYyMGZkNDc5MTQwNDU1Mzg4Zjk5NDIwZmQzMDczNjM6NDkyYzE4ZGNkYjBhNDNjNWJiMTBjYz" \
  "FjZDIxN2U4MDI="
#define DEFAULT_PCM_CAPTURE_LEN (2048)

#define I2S_SAMPLE_RATE 16000
#define I2S_CHANNELS 2
#define I2S_BITS 16

#define CODEC_CHANNELS 1
#define ESP_READ_BUFFER_SIZE 1024

#define DEFAULT_MAX_BITRATE (2000000)

typedef struct {
  bool b_wifi_connected;
  bool b_login_success;
  bool b_call_session_started;
} app_t;

static const char *TAG = "Agora";
static const char *cert_for_test =
    "eyJzaWduIjoiTS9FMkI3RFBqM09CdDFhZFBDNXhLM1BXa2Uva1llYko1blhDWk01cVNLT3o3Mm"
    "VuZ05jMUNSSXNRN"
    "HE3Ym53aE55aVFITThFV241d1R2ZUhsNVkrNGpMWVYybGI0QmQ5VDJPYU15UXkxTTQ3V3Z4Vk5"
    "CdmxEZEs3OEZBS1U"
    "yaWlFODFQNGV5VkxGL2ZackNxMWRZTytSc3I2N2ZpdEZYdXpGd2pGU1dVR1lPRG1FSjltYllxR"
    "k13dXR4VWgzNFZqV"
    "WVBT2dreFdsTndvbXlrMGF6R0ZQWSszVDRBNVdENTNhSytZeE5XUFFRQ1EvaExLNkRqRmZmMDl"
    "Cbmc4WTlCZi9XSmY"
    "5MjRISzB6dFdWTXZ4bUVrMHBBbmFvQTAwNC8za1gwdkU0SEJ6eDVOZWNpblk2K2YzcFJPUHJWO"
    "UpzUmh3MEQxWEtKM"
    "kFoUFR3ZERDeWJRNlpRPT0iLCJjdXN0b20iOiIyMDIyLTAzLTExVDE4OjI1OjQ3KzA4OjAwIiw"
    "iY3JlZGVudGlhbCI"
    "6IjRhNWZhMmVlZGRkMmIyYzljN2M0MTk5YWFkNDgyMDViOWIwYzliYjE5ZWFlZmRlYTQ5OTZhY"
    "Tk5ZjZkZDkyNTIiL"
    "CJkdWUiOiIyMDIyMDQxNyJ9";

static app_t g_app = {
    .b_login_success = false,
    .b_call_session_started = false,
    .b_wifi_connected = false,
};

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
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
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_YUV422,  // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = CONFIG_FRAME_SIZE,   // QQVGA-UXGA Do not use sizes above QVGA
                                      // when not JPEG

    .jpeg_quality = 12,  // 0-63 lower number means higher quality
    .fb_count =
        2,  // if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static agora_iot_account g_self = {
    .name = CONFIG_DEVICE_ID, .type = TYPE_DEVICE, .uid = 0};
static agora_iot_handle_t g_handle = NULL;

static uint32_t image_cnt = 0;
static uint32_t tick_begin = 0;
static esp_timer_handle_t fps_timer = NULL;

static void *jpg_encoder;
static audio_element_handle_t raw_read, raw_write, element_algo;
static audio_pipeline_handle_t recorder, player;
// static ringbuf_handle_t ringbuf_r, ringbuf_w;

static esp_err_t es7210_write_reg(i2c_bus_handle_t i2c_handle, uint8_t reg_addr,
                                  uint8_t data) {
  return i2c_bus_write_bytes(i2c_handle, ES7210_AD1_AD0_00, &reg_addr,
                             sizeof(reg_addr), &data, sizeof(data));
}

static void set_es7210_tdm_mode() {
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

static void fps_timer_callback(void *arg) {
  uint32_t cur_tick = xTaskGetTickCount();
  uint32_t duration = cur_tick - tick_begin;
  ESP_LOGW(TAG, "duration %-15u, image cnt %-10u, fps %f", duration, image_cnt,
           (float)image_cnt / (float)(duration / CONFIG_FREERTOS_HZ));

  audio_sys_get_real_time_stats();
  ESP_LOGI(TAG, "MEM Total:%d Bytes, Inter:%d Bytes, Dram:%d Bytes",
           esp_get_free_heap_size(),
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}

static esp_err_t input_key_service_cb(periph_service_handle_t handle,
                                      periph_service_event_t *evt, void *ctx) {
  if (evt->type == INPUT_KEY_SERVICE_ACTION_CLICK) {
    switch ((int)evt->data) {
      case INPUT_KEY_USER_ID_REC: {
        agora_iot_account peer = {
            .type = TYPE_USER,
            .name = CONFIG_USER_ACCOUNT,
        };
        ESP_LOGW(TAG, "Now we're calling \"%s\" ...", peer.name);
        agora_iot_call(g_handle, &peer, 1, "I'm the guest");
      } break;
      case INPUT_KEY_USER_ID_VOLUP:
        ESP_LOGW(TAG, "Hang up the call.");
        agora_iot_hang_up(g_handle);
        g_app.b_call_session_started = false;
        break;
      default:
        ESP_LOGE(TAG, "User Key ID[%d] does not support", (int)evt->data);
        break;
    }
  }

  return ESP_OK;
}

static void start_key_service(void) {
  ESP_LOGI(TAG, "Initialize peripherals");
  esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
  esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

  ESP_LOGI(TAG, "Initialize Button peripheral with board init");
  audio_board_key_init(set);

  ESP_LOGI(TAG, "Create and start input key service");
  input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
  input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
  input_cfg.handle = set;
  input_cfg.based_cfg.task_stack = 5 * 1024;
  periph_service_handle_t input_ser = input_key_service_create(&input_cfg);

  input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
  periph_service_set_callback(input_ser, input_key_service_cb, NULL);
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    g_app.b_wifi_connected = true;
    ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
  }
}

/*init wifi as sta and set power save mode*/
static void setup_wifi(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = CONFIG_WIFI_SSID,
              .password = CONFIG_WIFI_PASSWORD,
              .listen_interval = DEFAULT_LISTEN_INTERVAL,
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "esp_wifi_set_ps().");
#ifdef CONFIG_ENABLE_LOW_POWER_MODE
  esp_wifi_set_ps(DEFAULT_PS_MODE);
#else
  esp_wifi_set_ps(WIFI_PS_NONE);
#endif
}

static esp_err_t i2s_data_divided(int16_t *raw_buff, int len,
                                  int16_t *buf_aec) {
  for (int i = 0; i < len / 8; i++) {
    buf_aec[i << 1] = raw_buff[(i << 2) + 1];
    buf_aec[(i << 1) + 1] = raw_buff[i << 2];
  }
  return ESP_OK;
}

int i2s_stream_read_cb(audio_element_handle_t el, char *buf, int len,
                       TickType_t wait_time, void *ctx) {
  size_t bytes_read = 0;

  char *buf_tmp = audio_calloc(1, ESP_READ_BUFFER_SIZE);
  AUDIO_MEM_CHECK(TAG, buf, return 0);
  char *buf_aec = audio_calloc(1, ESP_READ_BUFFER_SIZE / 2);
  AUDIO_MEM_CHECK(TAG, buf_aec, return 0);

  i2s_read(0, buf_tmp, ESP_READ_BUFFER_SIZE, &bytes_read, wait_time);
  if (bytes_read == ESP_READ_BUFFER_SIZE) {
    i2s_data_divided((int16_t *)buf_tmp, ESP_READ_BUFFER_SIZE,
                     (int16_t *)buf_aec);
    bytes_read = ESP_READ_BUFFER_SIZE / 2;
    memcpy(buf, buf_aec, bytes_read);
  }

  free(buf_tmp);
  free(buf_aec);

  return bytes_read;
}

static esp_err_t recorder_pipeline_open() {
  audio_element_handle_t i2s_stream_reader;
  audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
  recorder = audio_pipeline_init(&pipeline_cfg);
  AUDIO_NULL_CHECK(TAG, recorder, return ESP_FAIL);

  i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
  i2s_cfg.type = AUDIO_STREAM_READER;
  i2s_cfg.uninstall_drv = false;
#ifdef CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
  i2s_cfg.i2s_port = 1;
#endif
  i2s_cfg.task_core = 1;
  i2s_cfg.i2s_config.sample_rate = I2S_SAMPLE_RATE;
#ifdef CONFIG_ESP32_S3_KORVO2_V3_BOARD
  i2s_cfg.i2s_config.bits_per_sample = 32;
#endif
  i2s_cfg.out_rb_size = 2 * 1024;
  i2s_cfg.stack_in_ext = true;
  i2s_stream_reader = i2s_stream_init(&i2s_cfg);
#ifdef CONFIG_ESP32_S3_KORVO2_V3_BOARD
  audio_element_set_read_cb(i2s_stream_reader, i2s_stream_read_cb, NULL);
#endif

  algorithm_stream_cfg_t algo_config = ALGORITHM_STREAM_CFG_DEFAULT();
#if defined(CONFIG_ESP_LYRAT_MINI_V1_1_BOARD) || \
    defined(CONFIG_ESP32_S3_KORVO2_V3_BOARD)
  algo_config.input_type = ALGORITHM_STREAM_INPUT_TYPE1;
#else
  algo_config.input_type = ALGORITHM_STREAM_INPUT_TYPE2;
#endif
  // algo_config.task_core = 1;
  algo_config.task_stack = 4 * 1024;
  algo_config.algo_mask = ALGORITHM_STREAM_USE_AEC;

  element_algo = algo_stream_init(&algo_config);
  audio_element_set_music_info(element_algo, I2S_SAMPLE_RATE, 1, I2S_BITS);

  audio_pipeline_register(recorder, i2s_stream_reader, "i2s");
  audio_pipeline_register(recorder, element_algo, "algo");

  raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
  raw_cfg.type = AUDIO_STREAM_READER;
  raw_cfg.out_rb_size = 2 * 1024;
  raw_read = raw_stream_init(&raw_cfg);
  audio_element_set_output_timeout(raw_read, portMAX_DELAY);

  audio_pipeline_register(recorder, raw_read, "raw");

  const char *link_tag[3] = {"i2s", "algo", "raw"};
  audio_pipeline_link(recorder, &link_tag[0], 3);

  ESP_LOGI(TAG, "audio recorder has been created");
  return ESP_OK;
}

static esp_err_t player_pipeline_open() {
  audio_element_handle_t i2s_stream_writer;
  audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
  player = audio_pipeline_init(&pipeline_cfg);
  AUDIO_NULL_CHECK(TAG, player, return ESP_FAIL);

  raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
  raw_cfg.type = AUDIO_STREAM_WRITER;
  raw_cfg.out_rb_size = 8 * 1024;
  raw_write = raw_stream_init(&raw_cfg);

  rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
  rsp_cfg.src_rate = I2S_SAMPLE_RATE;
  rsp_cfg.src_ch = CODEC_CHANNELS;
  rsp_cfg.dest_rate = I2S_SAMPLE_RATE;
  rsp_cfg.dest_ch = I2S_CHANNELS;
  rsp_cfg.complexity = 5;
  rsp_cfg.task_core = 1;
  rsp_cfg.out_rb_size = 8 * 1024;
  audio_element_handle_t filter = rsp_filter_init(&rsp_cfg);

  i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
  i2s_cfg.type = AUDIO_STREAM_WRITER;
  i2s_cfg.uninstall_drv = false;
  i2s_cfg.i2s_config.sample_rate = I2S_SAMPLE_RATE;
#ifdef CONFIG_ESP32_S3_KORVO2_V3_BOARD
  i2s_cfg.i2s_config.bits_per_sample = 32;
  i2s_cfg.need_expand = true;
#endif
#if !defined(CONFIG_ESP_LYRAT_MINI_V1_1_BOARD) && \
    !defined(CONFIG_ESP32_S3_KORVO2_V3_BOARD)
  i2s_cfg.multi_out_num = 1;
#endif
  i2s_cfg.task_core = 1;
  i2s_cfg.stack_in_ext = true;
  i2s_stream_writer = i2s_stream_init(&i2s_cfg);

  audio_pipeline_register(player, raw_write, "raw");
  audio_pipeline_register(player, filter, "filter");
  audio_pipeline_register(player, i2s_stream_writer, "i2s");
  const char *link_tag[3] = {"raw", "filter", "i2s"};
  audio_pipeline_link(player, &link_tag[0], 3);

  return ESP_OK;
}

static void setup_audio(void) {
  audio_board_handle_t board_handle = audio_board_init();
  audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH,
                       AUDIO_HAL_CTRL_START);
  audio_hal_set_volume(board_handle->audio_hal, 70);

  es7210_mic_select(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2 | ES7210_INPUT_MIC3 |
                    ES7210_INPUT_MIC4);
  set_es7210_tdm_mode();

  recorder_pipeline_open();
  player_pipeline_open();
}

static void init_camera(void) {
  // initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera Init Failed");
  }
}

static void *init_jpeg_encoder(int quality, int hfm_core, int hfm_priority,
                               jpeg_subsampling_t subsampling) {
  jpeg_enc_info_t jpeg_enc_info = {0};
  jpeg_enc_info.width = CAMERA_WIDTH;
  jpeg_enc_info.height = CAMERA_HIGH;
  jpeg_enc_info.src_type = JPEG_RAW_TYPE_YCbYCr;
  jpeg_enc_info.subsampling = subsampling;
  jpeg_enc_info.quality = quality;
  // jpeg_enc_info.task_enable = true;
  jpeg_enc_info.hfm_task_core = hfm_core;
  jpeg_enc_info.hfm_task_priority = hfm_priority;
  return jpeg_enc_open(&jpeg_enc_info);
}

/**
 * Agora video call related functions
 */
#ifdef CONFIG_REGISTER_NEW_DEVICE
static int register_new_device(const char *device_id) {
  agora_iot_account account = {0};
  int rval;

  account.type = TYPE_DEVICE;
  strncpy((char *)&account.name, device_id, sizeof(account.name));
  rval = agora_iot_register(CONFIG_AGORA_APP_ID, &account);
  if (rval < 0) {
    ESP_LOGE(TAG, "Failed to register device ID %s, err code %d", account.name,
             rval);
    return -1;
  }

  ESP_LOGI(TAG, "Register device ID \"%s\" successfully.", account.name);
  return 0;
}
#endif

static int send_video_frame(uint8_t *data, uint32_t len) {
  int rval;

  // API: send video data
  ago_video_frame_t ago_frame = {0};
  ago_frame.data_type = AGO_VIDEO_DATA_TYPE_JPEG;
  ago_frame.is_key_frame = true;
  ago_frame.video_buffer = data;
  ago_frame.video_buffer_size = len;
  rval = agora_iot_push_video_frame(g_handle, &ago_frame);
  if (rval < 0) {
    ESP_LOGE(TAG, "Failed to push video frame");
    return -1;
  }

  return 0;
}

static int send_audio_frame(uint8_t *data, uint32_t len) {
  int rval;

  // API: send audio data
  ago_audio_frame_t ago_frame = {0};
  ago_frame.data_type = AGO_AUDIO_DATA_TYPE_PCM;
  ago_frame.audio_buffer = data;
  ago_frame.audio_buffer_size = len;
  rval = agora_iot_push_audio_frame(g_handle, &ago_frame);
  if (rval < 0) {
    ESP_LOGE(TAG, "Failed to push audio frame");
    return -1;
  }

  return 0;
}

static void video_capture_and_send_task(void *args) {
  const int image_buf_len = 30 * 1024;
  int image_len = 0;

#ifdef CONFIG_ENABLE_RUN_TIME_STATS
  esp_timer_create_args_t create_args = {
      .callback = fps_timer_callback, .arg = NULL, .name = "fps timer"};
  esp_timer_create(&create_args, &fps_timer);
  esp_timer_start_periodic(fps_timer, 5 * 1000 * 1000);
  tick_begin = xTaskGetTickCount();
  image_cnt = 0;
#endif

  uint8_t *image_buf = heap_caps_malloc(image_buf_len, MALLOC_CAP_SPIRAM);
  if (!image_buf) {
    ESP_LOGE(TAG, "Failed to alloc video buffer!");
    return;
  }

  while (g_app.b_call_session_started) {
    camera_fb_t *pic = esp_camera_fb_get();
    image_cnt++;

    jpeg_enc_process(jpg_encoder, pic->buf, pic->len, image_buf, image_buf_len,
                     &image_len);

    // ESP_LOGI(TAG, "YUV len %d, JPEG len %d", pic->len, image_len);
    send_video_frame(image_buf, image_len);

    esp_camera_fb_return(pic);
  }
  free(image_buf);

#ifdef CONFIG_ENABLE_RUN_TIME_STATS
  esp_timer_stop(fps_timer);
  esp_timer_delete(fps_timer);
#endif
  vTaskDelete(NULL);
}

static void audio_capture_and_send_task(void *threadid) {
  int read_len = DEFAULT_PCM_CAPTURE_LEN;
  int ret;

  uint8_t *pcm_buf =
      heap_caps_malloc(read_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!pcm_buf) {
    ESP_LOGE(TAG, "Failed to alloc audio buffer!");
    return;
  }

  audio_pipeline_run(recorder);
  audio_pipeline_run(player);
  while (g_app.b_call_session_started) {
    ret = raw_stream_read(raw_read, (char *)pcm_buf, read_len);
    if (ret != read_len) {
      ESP_LOGW(TAG, "write error, expect %d, but only %d", read_len, ret);
    }
    send_audio_frame(pcm_buf, DEFAULT_PCM_CAPTURE_LEN);
  }
  free(pcm_buf);
  vTaskDelete(NULL);
}

static void iot_cb_login_success(void) {
  ESP_LOGI(TAG, "Login agora iot server successfully");
  g_app.b_login_success = true;
}

static void iot_cb_call_request(const char *peer_name, const char *attach_msg) {
  if (!peer_name) {
    return;
  }

  ESP_LOGI(TAG, "Get call from peer \"%s\", attach message: %s", peer_name,
           attach_msg ? attach_msg : "null");

  agora_iot_answer(NULL);
}

static void iot_cb_start_push_frame() {
  int rval;

  ESP_LOGI(TAG, "Start push audio/video frames");
  g_app.b_call_session_started = true;

  rval = xTaskCreatePinnedToCore(video_capture_and_send_task, "video_task",
                                 3 * 1024, NULL, PRIO_TASK_FETCH, NULL, 1);
  if (rval != pdTRUE) {
    ESP_LOGE(TAG, "Unable to create video capture thread!");
    return;
  }

  rval = xTaskCreatePinnedToCore(audio_capture_and_send_task, "audio_task",
                                 3 * 1024, NULL, PRIO_TASK_FETCH, NULL, 1);
  if (rval != pdTRUE) {
    ESP_LOGE(TAG, "Unable to create audio capture thread!");
    return;
  }
}

static void iot_cb_call_hung_up(const char *peer_name) {
  if (!peer_name) {
    return;
  }

  ESP_LOGI(TAG, "Get hangup from peer \"%s\"", peer_name);
  g_app.b_call_session_started = false;
}

static void iot_cb_call_answered(const char *peer_name) {
  ESP_LOGI(TAG, "Get answer from peer \"%s\"", peer_name);
}

static void iot_cb_receive_audio(ago_audio_frame_t *frame) {
  raw_stream_write(raw_write, (char *)frame->audio_buffer,
                   frame->audio_buffer_size);
}

int app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

#ifdef CONFIG_ENABLE_LOW_POWER_MODE
  esp_pm_config_esp32s3_t pm_config = {
      .max_freq_mhz = CONFIG_EXAMPLE_MAX_CPU_FREQ_MHZ,
      .min_freq_mhz = CONFIG_EXAMPLE_MIN_CPU_FREQ_MHZ,
      .light_sleep_enable = true};
  ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif
  setup_wifi();

  setup_audio();
  init_camera();

  jpg_encoder = init_jpeg_encoder(40, 0, 20, JPEG_SUB_SAMPLE_YUV420);
  if (!jpg_encoder) {
    ESP_LOGE(TAG, "Failed to initialize jpeg encoder!");
    return -1;
  }

  // Wait until WiFi is connected
  while (!g_app.b_wifi_connected) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

#ifdef CONFIG_REGISTER_NEW_DEVICE
  // Register the new device
  register_new_device(CONFIG_DEVICE_ID);
#endif

  // Initialize Agora IoT SDK
  agora_iot_callback_t cb = {
      .cb_login_success = iot_cb_login_success,
      .cb_call_request = iot_cb_call_request,
      .cb_start_push_frame = iot_cb_start_push_frame,
      .cb_call_hung_up = iot_cb_call_hung_up,
      .cb_call_answered = iot_cb_call_answered,
      .cb_receive_audio_frame = iot_cb_receive_audio,
  };

  agora_iot_audio_config_t audio_config = {
      .audio_codec_type = AGO_AUDIO_CODEC_TYPE_G722,
      .pcm_sample_rate = 16000,
      .pcm_channel_num = 1,
  };
  agora_iot_config_t cfg = {
      .app_id = CONFIG_AGORA_APP_ID,
      .cloud_rec_basic_auth = BASIC_AUTH,
      .product_id = "test_product",
      .device_id = CONFIG_DEVICE_ID,
      .region = 0,
      .self = &g_self,
      .enable_rtc = true,
      .disable_rtc_log = true,
      .max_possible_bitrate = DEFAULT_MAX_BITRATE,
      .certificate = cert_for_test,
      .enable_recv_audio = true,
      .enable_recv_video = true,
      .cb = &cb,
      .audio_config = &audio_config,
  };

  g_handle = agora_iot_init(&cfg);
  if (!g_handle) {
    // if failed to initialize, no need to deinitialize
    ESP_LOGE(TAG, "Failed to initialize Agoro IoT");
    return -1;
  }

  // Wait until login Agora IoT serivce successfully
  while (!g_app.b_login_success) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  // Monitor the "REC" key event
  start_key_service();

  // Infinite loop
  while (1) {
    if (!g_app.b_call_session_started) {
      ESP_LOGW(TAG, "Please press [REC] key to ring the doorbell ...");
    } else {
      ESP_LOGW(TAG,
               "Now we're in the call. Please press [VOL+] key to hang up ...");
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }

  // Deinit Agora IoT SDK
  agora_iot_deinit(g_handle);

  return 0;
}
