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
#include <sys/param.h>
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
#ifdef CONFIG_ESP32_S3_KORVO2_V3_BOARD
#include "es7210.h"
#endif
#include "esp_err.h"
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
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "audio_thread.h"
#ifdef UVC_STREAM_ENABLE
#include "uvc_stream.h"
#else
#include "esp_camera.h"
#include "esp_jpeg_enc.h"
#endif
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

#define PRIO_TASK_FETCH (21)

#define I2S_CHANNELS 1
#define I2S_BITS 16

#define ESP_READ_BUFFER_SIZE 320

#define BWE_MIN_BPS (50 * 1000)
#define BWE_MAX_BPS (1500 * 1000)
#define BASE_VIDEO_FPS 10

#define CONFIG_CONTENT_LEN   256


typedef struct {
  bool b_wifi_connected;
  bool b_call_session_started;
  bool b_exit;
  sys_up_mode_e up_mode;

  uint32_t total_target_send_bps;
  uint32_t video_send_target_video_frame_rate;
  uint32_t video_latest_2_second_send_bps;
  uint8_t  video_capture_fps;
} app_t;

static app_t g_app = {
    .b_call_session_started = false,
    .b_wifi_connected       = false,
    .b_exit                 = false,
    .up_mode                = SYS_UP_MODE_POWERON,
    .video_capture_fps      = BASE_VIDEO_FPS,
    .video_latest_2_second_send_bps = BWE_MIN_BPS,
    .video_send_target_video_frame_rate = BASE_VIDEO_FPS,
    .total_target_send_bps   = BWE_MAX_BPS,
};

#ifdef UVC_STREAM_ENABLE
/* USB Camera Descriptors Related MACROS,
the quick demo skip the standred get descriptors process,
users need to get params from camera descriptors from PC side,
eg. run `lsusb -v` in linux,
then hardcode the related MACROS below
*/
#define DESCRIPTOR_CONFIGURATION_INDEX 1
#define DESCRIPTOR_FORMAT_MJPEG_INDEX  2

#define DESCRIPTOR_FRAME_640_480_INDEX 1
#define DESCRIPTOR_FRAME_352_288_INDEX 2
#define DESCRIPTOR_FRAME_320_240_INDEX 3
#define DESCRIPTOR_FRAME_160_120_INDEX 4

#define DESCRIPTOR_FRAME_5FPS_INTERVAL  2000000
#define DESCRIPTOR_FRAME_10FPS_INTERVAL 1000000
#define DESCRIPTOR_FRAME_15FPS_INTERVAL 666666
#define DESCRIPTOR_FRAME_30FPS_INTERVAL 333333

#define DESCRIPTOR_STREAM_INTERFACE_INDEX   1
#define DESCRIPTOR_STREAM_INTERFACE_ALT_MPS_128 1
#define DESCRIPTOR_STREAM_INTERFACE_ALT_MPS_256 2
#define DESCRIPTOR_STREAM_INTERFACE_ALT_MPS_512 3
#define DESCRIPTOR_STREAM_INTERFACE_BULK        0

#define DESCRIPTOR_STREAM_ISOC_ENDPOINT_ADDR 0x81

/* Demo Related MACROS */
#if (FRAMESIZE == 0)
/* Isochronous transfer mode config */
#define DEMO_XFER_MODE        UVC_XFER_ISOC
#define DEMO_FRAME_WIDTH      320
#define DEMO_FRAME_HEIGHT     240
#define DEMO_XFER_BUFFER_SIZE (35 * 1024) //Double buffer
#define DEMO_FRAME_INDEX      DESCRIPTOR_FRAME_320_240_INDEX
#define DEMO_FRAME_INTERVAL   DESCRIPTOR_FRAME_15FPS_INTERVAL
#define DEMO_INTERFACE_ALT    DESCRIPTOR_STREAM_INTERFACE_ALT_MPS_512
#define DEMO_EP_MPS           512         // max MPS of esp32-s2/s3 is 1*512
#elif (FRAMESIZE == 2)
/* Bulk transfer mode config */
#define DEMO_XFER_MODE        UVC_XFER_BULK
#define DEMO_FRAME_WIDTH      640
#define DEMO_FRAME_HEIGHT     480
#define DEMO_XFER_BUFFER_SIZE (48 * 1024) //Double buffer
#define DEMO_FRAME_INDEX      DESCRIPTOR_FRAME_640_480_INDEX
#define DEMO_FRAME_INTERVAL   DESCRIPTOR_FRAME_15FPS_INTERVAL
#define DEMO_INTERFACE_ALT    DESCRIPTOR_STREAM_INTERFACE_BULK
#define DEMO_EP_MPS           64
#endif

#define BIT1_NEW_FRAME_START (0x01 << 1)
#define BIT2_NEW_FRAME_END (0x01 << 2)
static EventGroupHandle_t s_evt_handle;
static camera_fb_t s_fb = {0};

#else //UVC_STREAM_ENABLE

#if (FRAMESIZE == 0)
#define CONFIG_FRAME_SIZE (FRAMESIZE_QVGA)
#define CONFIG_FRAME_WIDTH 320
#define CONFIG_FRAME_HIGH 240
#elif (FRAMESIZE == 1)
#define CONFIG_FRAME_SIZE (FRAMESIZE_HVGA)
#define CONFIG_FRAME_WIDTH 480
#define CONFIG_FRAME_HIGH 320
#else
#define CONFIG_FRAME_SIZE (FRAMESIZE_VGA)
#define CONFIG_FRAME_WIDTH 640
#define CONFIG_FRAME_HIGH 480
#endif

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

static void *jpg_encoder;
#endif

static agora_iot_handle_t g_handle = NULL;
static device_handle_t dev_state = NULL;

static uint32_t image_cnt = 0;
static uint32_t tick_begin = 0;
static esp_timer_handle_t fps_timer = NULL;

static audio_element_handle_t raw_read, raw_write, element_algo;
static audio_pipeline_handle_t recorder, player;

static SemaphoreHandle_t g_video_capture_sem  = NULL;
static audio_thread_t *g_video_thread;

static SemaphoreHandle_t g_audio_capture_sem  = NULL;
static audio_thread_t *g_audio_thread;

#ifndef CONFIG_BLUFI_ENABLE
static QueueHandle_t    g_qr_queue;
#endif

static uint8_t g_push_type = 0x00;

static void __calc_latest_2_sec_video_data_bps(uint32_t video_data_len)
{
    static int64_t last_time = 0;
    static uint32_t total_bytes_len = 0;
    int64_t        now;

    //init last time
    if (0 == last_time) {
        last_time = esp_timer_get_time();
    }

    total_bytes_len += video_data_len;
    now              = esp_timer_get_time();

    // ESP_LOGI(TAG, "last_time = %lld, total_bytes_len = %u, now = %lld, video_data_len = %u", last_time, total_bytes_len, now, video_data_len);
    // //uint32_t wrap around
    // if (now < last_time) {
    //     total_bytes_len = 0;
    //     last_time       = now;
    //     return;
    // }
    int32_t diff = (now - last_time) / 1000;
    if (diff >= 2000) {
        g_app.video_latest_2_second_send_bps = (total_bytes_len << 13) / diff;
        total_bytes_len                      = 0;
        last_time                            = now;
        // ESP_LOGI(TAG, "video_latest_2_second_send_bps = %u", g_app.video_latest_2_second_send_bps);
    }
}

#define DIV_ROUND(divident, divider)    (((divident) + ((divider) >> 1)) / (divider))
static uint32_t __is_send_bps_adjust_need_skip_this_frame(void)
{
    static int32_t val = BASE_VIDEO_FPS;
    int32_t skip = 1;

    if (g_app.total_target_send_bps >= g_app.video_latest_2_second_send_bps) {
        g_app.video_send_target_video_frame_rate = g_app.video_capture_fps;
        // ESP_LOGI(TAG, "bwe enough, donnot need skip frame[%u, %u], g_app.video_capture_fps %u, val %u",
                // g_app.total_target_send_bps, g_app.video_latest_2_second_send_bps, g_app.video_capture_fps, val);
        return 0;
    }

    g_app.video_send_target_video_frame_rate = DIV_ROUND((g_app.total_target_send_bps * g_app.video_capture_fps), g_app.video_latest_2_second_send_bps);
    if (val >= g_app.video_capture_fps) {
        skip = 0;
        val -= g_app.video_capture_fps;
    }

    val += g_app.video_send_target_video_frame_rate;
    // ESP_LOGI(TAG, "vps=%u, skip=%d, target=%u, latest=%u",
    //      g_app.video_send_target_video_frame_rate, skip,
    //      g_app.total_target_send_bps, g_app.video_latest_2_second_send_bps);

    return skip;
}

#ifdef CONFIG_ESP32_S3_KORVO2_V3_BOARD
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
#endif

static void fps_timer_callback(void *arg)
{
  if (g_app.b_call_session_started && image_cnt) {
    uint32_t cur_tick = xTaskGetTickCount();
    uint32_t duration = cur_tick - tick_begin;

    ESP_LOGW(TAG, "duration %-15u, image cnt %-10u, fps %f", duration, image_cnt,
              (float)image_cnt / (float)(duration / CONFIG_FREERTOS_HZ));

    image_cnt  = 0;
    tick_begin = cur_tick;
  }

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
    case INPUT_KEY_USER_ID_MUTE: {
      agora_iot_file_info_t file_info = {
        .name_suffix  = "jpeg",
        .buf          = NULL,
        .size         = 0
      };
      alarm_message_send(g_handle, file_info, "nick_esp32", AG_ALARM_TYPE_MOD, "This is a alarm test");
      int ret = start_alarm_record(g_handle);
      ESP_LOGW(TAG, "start_alarm_record %d...", ret);
    } break;
    case INPUT_KEY_USER_ID_PLAY: {
      int ret = stop_alarm_record(g_handle);
      ESP_LOGW(TAG, "stop_alarm_record %d...", ret);
    } break;
    default:
      ESP_LOGE(TAG, "User Key ID[%d] does not support", (int)evt->data);
      break;
    }
  }

  return ESP_OK;
}

static void start_key_service(esp_periph_set_handle_t set)
{
  ESP_LOGI(TAG, "Initialize Button peripheral with board init");
  audio_board_key_init(set);

  ESP_LOGI(TAG, "Create and start input key service");
  input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
  input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
  input_cfg.handle                 = set;
  input_cfg.based_cfg.task_stack   = 8 * 1024;
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
  i2s_cfg.i2s_config.sample_rate     = CONFIG_PCM_SAMPLE_RATE;
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
  algo_config.algo_mask  = ALGORITHM_STREAM_USE_AEC;
  // algo_config.aec_low_cost = true;

  element_algo = algo_stream_init(&algo_config);
  audio_element_set_music_info(element_algo, CONFIG_PCM_SAMPLE_RATE, 1, I2S_BITS);

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
  i2s_cfg.i2s_config.sample_rate     = CONFIG_PCM_SAMPLE_RATE;
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

#ifdef CONFIG_ESP32_S3_KORVO2_V3_BOARD
  es7210_mic_select(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2 | ES7210_INPUT_MIC3 | ES7210_INPUT_MIC4);
  set_es7210_tdm_mode();
#endif
}

#ifndef CONFIG_AUDIO_ONLY
#ifdef UVC_STREAM_ENABLE
static camera_fb_t* esp_camera_fb_get()
{
  xEventGroupWaitBits(s_evt_handle, BIT1_NEW_FRAME_START, true, true, portMAX_DELAY);
  ESP_LOGD(TAG, "peek frame = %ld", s_fb.timestamp.tv_sec);
  return &s_fb;
}

static void esp_camera_fb_return(camera_fb_t * fb)
{
  ESP_LOGD(TAG, "release frame = %ld", fb->timestamp.tv_sec);
  xEventGroupSetBits(s_evt_handle, BIT2_NEW_FRAME_END);
}

/* *******************************************************************************************
 * This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
static void frame_cb(uvc_frame_t *frame, void *ptr)
{
  ESP_LOGD(TAG, "callback! frame_format = %d, seq = %u, width = %d, height = %d, length = %u, ptr = %d",
           frame->frame_format, frame->sequence, frame->width, frame->height, frame->data_bytes, (int) ptr);

  switch (frame->frame_format) {
    case UVC_FRAME_FORMAT_MJPEG:
      s_fb.buf              = frame->data;
      s_fb.len              = frame->data_bytes;
      s_fb.width            = frame->width;
      s_fb.height           = frame->height;
      s_fb.format           = PIXFORMAT_JPEG;
      s_fb.timestamp.tv_sec = frame->sequence;
      xEventGroupSetBits(s_evt_handle, BIT1_NEW_FRAME_START);
      xEventGroupWaitBits(s_evt_handle, BIT2_NEW_FRAME_END, true, true, pdTICKS_TO_MS(1000));
      break;
    default:
      ESP_LOGW(TAG, "Format not supported");
      assert(0);
      break;
  }
}

static void setup_uvc_stream(void)
{
  audio_board_usb_cam_init();

  /* create eventgroup for task sync */
  s_evt_handle = xEventGroupCreate();
  if (s_evt_handle == NULL) {
    ESP_LOGE(TAG, "line-%u event group create faild", __LINE__);
    assert(0);
  }

  /* malloc double buffer for usb payload, xfer_buffer_size >= frame_buffer_size */
  uint8_t *xfer_buffer_a = (uint8_t *)heap_caps_malloc(DEMO_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  assert(xfer_buffer_a != NULL);
  uint8_t *xfer_buffer_b = (uint8_t *)heap_caps_malloc(DEMO_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  assert(xfer_buffer_b != NULL);

  /* malloc frame buffer for a jpeg frame*/
  uint8_t *frame_buffer = (uint8_t *)heap_caps_malloc(DEMO_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  assert(frame_buffer != NULL);

  /* the quick demo skip the standred get descriptors process,
  users need to get params from camera descriptors from PC side,
  eg. run `lsusb -v` in linux, then modify related MACROS */
  uvc_config_t uvc_config = {
    .xfer_type         = UVC_XFER_ISOC,
    .dev_speed         = USB_SPEED_FULL,
    .configuration     = DESCRIPTOR_CONFIGURATION_INDEX,
    .format_index      = DESCRIPTOR_FORMAT_MJPEG_INDEX,
    .frame_width       = DEMO_FRAME_WIDTH,
    .frame_height      = DEMO_FRAME_HEIGHT,
    .frame_index       = DEMO_FRAME_INDEX,
    .frame_interval    = DEMO_FRAME_INTERVAL,
    .interface         = DESCRIPTOR_STREAM_INTERFACE_INDEX,
    .interface_alt     = DEMO_INTERFACE_ALT,
    .isoc_ep_addr      = DESCRIPTOR_STREAM_ISOC_ENDPOINT_ADDR,
    .isoc_ep_mps       = DEMO_EP_MPS,
    .xfer_buffer_size  = DEMO_XFER_BUFFER_SIZE,
    .xfer_buffer_a     = xfer_buffer_a,
    .xfer_buffer_b     = xfer_buffer_b,
    .frame_buffer_size = DEMO_XFER_BUFFER_SIZE,
    .frame_buffer      = frame_buffer,
  };

  /* pre-config UVC driver with params from known USB Camera Descriptors*/
  esp_err_t ret = uvc_streaming_config(&uvc_config);

  /* Start camera IN stream with pre-configs, uvc driver will create multi-tasks internal
  to handle usb data from different pipes, and user's callback will be called after new frame ready. */
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "uvc streaming config failed");
  } else {
    ret = uvc_streaming_start(frame_cb, NULL);
    uvc_streaming_suspend();
    ESP_LOGI(TAG, "uvc streaming setup success %d.", ret);
  }
}
#else //#ifdef UVC_STREAM_ENABLE
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
  jpeg_enc_info.width             = CONFIG_FRAME_WIDTH;
  jpeg_enc_info.height            = CONFIG_FRAME_HIGH;
  jpeg_enc_info.src_type          = JPEG_RAW_TYPE_YCbYCr;
  jpeg_enc_info.subsampling       = subsampling;
  jpeg_enc_info.quality           = quality;
  // jpeg_enc_info.task_enable     = true;
  jpeg_enc_info.hfm_task_core     = hfm_core;
  jpeg_enc_info.hfm_task_priority = hfm_priority;
  return jpeg_enc_open(&jpeg_enc_info);
}
#endif //#ifdef UVC_STREAM_ENABLE

static int send_video_frame(uint8_t *data, uint32_t len)
{
    int rval = -1;

    // API: send video data
    ago_video_frame_t ago_frame = { 0 };
    ago_frame.data_type         = AGO_VIDEO_DATA_TYPE_JPEG;
    ago_frame.is_key_frame      = true;
    ago_frame.video_buffer      = data;
    ago_frame.video_buffer_size = len;
    rval = agora_iot_push_video_frame(g_handle, &ago_frame, g_push_type);
    if (rval < 0) {
        ESP_LOGE(TAG, "Failed to push video frame");
        return -1;
    }

    return 0;
}

static void video_capture_and_send_task(void *args)
{
  const int image_buf_len = 30 * 1024;
  int image_len = 0;

  uint8_t *image_buf = heap_caps_malloc(image_buf_len, MALLOC_CAP_SPIRAM);
  if (!image_buf) {
    ESP_LOGE(TAG, "Failed to alloc video buffer!");
    return;
  }

  while (1) {
    xSemaphoreTake(g_video_capture_sem, portMAX_DELAY);

    while (g_app.b_call_session_started) {
      camera_fb_t *pic = esp_camera_fb_get();

#ifndef UVC_STREAM_ENABLE
      __calc_latest_2_sec_video_data_bps(image_len);
      if (!__is_send_bps_adjust_need_skip_this_frame()) {
        image_cnt++;
        jpeg_enc_process(jpg_encoder, pic->buf, pic->len, image_buf, image_buf_len, &image_len);
        send_video_frame(image_buf, image_len);
      }
#else
      __calc_latest_2_sec_video_data_bps(pic->len);
      if (!__is_send_bps_adjust_need_skip_this_frame()) {
        image_cnt++;
        send_video_frame(pic->buf, pic->len); 
      }
#endif

      vTaskDelay(20 / portTICK_PERIOD_MS);
      esp_camera_fb_return(pic);
    }
  }
  free(image_buf);

#ifndef UVC_STREAM_ENABLE
  if (jpg_encoder) {
    jpeg_enc_close(jpg_encoder);
  }
#endif

  vTaskDelete(NULL);
}
#endif

static int send_audio_frame(uint8_t *data, uint32_t len)
{
    int rval = -1;

    // API: send audio data
    ago_audio_frame_t ago_frame = { 0 };
    ago_frame.data_type         = AGO_AUDIO_DATA_TYPE_PCM;
    ago_frame.audio_buffer      = data;
    ago_frame.audio_buffer_size = len;
    rval = agora_iot_push_audio_frame(g_handle, &ago_frame, g_push_type);
    if (rval < 0) {
        ESP_LOGE(TAG, "Failed to push audio frame");
        return -1;
    }

    return 0;
}

static void audio_capture_and_send_task(void *threadid)
{
  int read_len = CONFIG_PCM_DATA_LEN;
  int ret;

  uint8_t *pcm_buf = heap_caps_malloc(read_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!pcm_buf) {
    ESP_LOGE(TAG, "Failed to alloc audio buffer!");
    return;
  }

  while (1) {
    recorder_pipeline_open();
    player_pipeline_open();

    xSemaphoreTake(g_audio_capture_sem, portMAX_DELAY);

    audio_pipeline_run(recorder);
    audio_pipeline_run(player);

    while (g_app.b_call_session_started) {
      ret = raw_stream_read(raw_read, (char *)pcm_buf, read_len);
      if (ret != read_len) {
        ESP_LOGW(TAG, "write error, expect %d, but only %d", read_len, ret);
      }
      send_audio_frame(pcm_buf, CONFIG_PCM_DATA_LEN);
    }

    //deinit
    audio_pipeline_stop(recorder);
    audio_pipeline_wait_for_stop(recorder);
    audio_pipeline_terminate(recorder);

    audio_pipeline_stop(player);
    audio_pipeline_wait_for_stop(player);
    audio_pipeline_terminate(player);
  }

  free(pcm_buf);
  vTaskDelete(NULL);
}

static void create_capture_task(void)
{
  esp_err_t rval = ESP_FAIL;

#ifndef CONFIG_AUDIO_ONLY
  g_video_capture_sem = xSemaphoreCreateBinary();
  if (NULL == g_video_capture_sem) {
    ESP_LOGE(TAG, "Unable to create video capture semaphore!");
    return;
  }
  rval = audio_thread_create(g_video_thread, "video_task", video_capture_and_send_task, NULL, 5 * 1024, PRIO_TASK_FETCH, true, 1);
  if (rval != ESP_OK) {
    ESP_LOGE(TAG, "Unable to create video capture thread!");
    return;
  }
#endif

  g_audio_capture_sem = xSemaphoreCreateBinary();
  if (NULL == g_audio_capture_sem) {
    ESP_LOGE(TAG, "Unable to create audio capture semaphore!");
    return;
  }
  rval = audio_thread_create(g_audio_thread, "audio_task", audio_capture_and_send_task, NULL, 5 * 1024, PRIO_TASK_FETCH, true, 0);
  if (rval != ESP_OK) {
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

static void iot_cb_start_push_frame(uint8_t push_type)
{
  ESP_LOGI(TAG, "Start push audio/video frames");
  g_app.b_call_session_started = true;

  // record push type
  g_push_type |= push_type;

#ifdef UVC_STREAM_ENABLE
  uvc_streaming_resume();
#endif //#ifdef UVC_STREAM_ENABLE

#ifndef CONFIG_AUDIO_ONLY
  xSemaphoreGive(g_video_capture_sem);
#endif

  xSemaphoreGive(g_audio_capture_sem);
}

static void iot_cb_stop_push_frame(uint8_t push_type)
{
  ESP_LOGI(TAG, "Stop push audio/video frames");
  g_app.b_call_session_started = false;

  // record push type
  g_push_type &= ~push_type;

#ifdef UVC_STREAM_ENABLE
  uvc_streaming_suspend();
#endif
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

static void iot_cb_target_bitrate_changed(uint32_t target_bps)
{
  printf("Bandwidth change detected. Please adjust encoder bitrate to %u kbps\n", target_bps / 1000);
  g_app.total_target_send_bps = target_bps;
}

static void iot_cb_key_frame_requested(void)
{
  // printf("Frame loss detected. Please notify the encoder to generate key frame immediately\n");
}

#ifndef CONFIG_BLUFI_ENABLE
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
      if (qd.payload_len > CONFIG_CONTENT_LEN) {
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
#endif

static int parse_config_content(device_handle_t dev_state, const char *content, char *ssid, char *pwd)
{
  int ret     = -1;

  cJSON *root = cJSON_Parse(content);
  if (NULL == root) {
    ESP_LOGE(TAG, "cannot parse QRcode: %s\n", content);
    goto _parse_err;
  }

  // get ssid
  cJSON *item = cJSON_GetObjectItemCaseSensitive(root, "s");
  ssid = cJSON_GetStringValue(item); 
  if (ssid != NULL) {
    device_set_item_string(dev_state, "ssid", ssid);
  } else {
    printf("#### cannot found ssid!\n");
  }
  item = NULL;

  // get pws
  item = cJSON_GetObjectItemCaseSensitive(root, "p");
  pwd = cJSON_GetStringValue(item);
  if (pwd != NULL) {
    device_set_item_string(dev_state, "password", pwd);
  } else {
    printf("#### cannot found password!\n");
  }
  item = NULL;

  // get product key
  item = cJSON_GetObjectItemCaseSensitive(root, "k");
  if (cJSON_GetStringValue(item)) {
    device_set_item_string(dev_state, "product_key", cJSON_GetStringValue(item));
  } else {
    printf("#### cannot found  product key!\n");
  }
  item = NULL;

  // get user id
  item = cJSON_GetObjectItemCaseSensitive(root, "u");
  if (cJSON_GetStringValue(item)) {
    device_set_item_string(dev_state, "user_id", cJSON_GetStringValue(item));
  } else {
    printf("#### cannot found user id!\n");
  }
  item = NULL;

  // get device name
  item = cJSON_GetObjectItemCaseSensitive(root, "n");
  if (cJSON_GetStringValue(item)) {
    device_set_item_string(dev_state, "device_name", cJSON_GetStringValue(item));
  } else {
    printf("#### cannot found device_name, use the default: %s !\n", get_device_id());
    device_set_item_string(dev_state, "device_name", get_device_id());
  }
  item = NULL;
  ret = 0;

_parse_err:
  if (root) {
    cJSON_Delete(root);
  }

  return ret;
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

static int agora_network_config(device_handle_t dev_state)
{
  char config_buff[CONFIG_CONTENT_LEN + 1] = { 0 };
  char ssid[32] = { 0 };
  char pwd[64]  = { 0 };

#ifdef CONFIG_BLUFI_ENABLE
  // setup wifi and Wait until WiFi is connected
extern void setup_wifi_with_block(char *cfg);
  setup_wifi_with_block(config_buff);

#if (LOWER_POWER_MODE == CONFIG_ENABLE_LIGHT_SLEEP)
  esp_wifi_set_ps(DEFAULT_PS_MODE);
#else
  esp_wifi_set_ps(WIFI_PS_NONE);
#endif

  return parse_config_content(dev_state, config_buff, ssid, pwd);
#else // CONFIG_BLUFI_ENABLE 
  ESP_LOGI(TAG, "\n\n------------------ Please input QRcode string with JSON type ----------------------\n");

  g_qr_queue = xQueueCreate(2, CONFIG_CONTENT_LEN);
  if (NULL == g_qr_queue) {
    ESP_LOGE(TAG, "create semaphore failed !\n");
    return -1;
  }

  // create the task for QR-Code
  xTaskCreate(qr_recoginze, "qr_recoginze_task", 1024 * 40, NULL, 5, NULL);

  // wait for the QR-Code from the qr_reconginze_task
  xQueueReceive(g_qr_queue, config_buff, portMAX_DELAY);

  ESP_LOGI(TAG, "qrcode context: %s\n", config_buff);
  ESP_LOGI(TAG, "-------------------- Got string and parse it now ------------------------------------\n");

  // parse content
  if (parse_config_content(dev_state, config_buff, ssid, pwd) == 0) {
    setup_wifi();
    return device_connect_network(ssid, pwd);
  } else
    return -1;
#endif // CONFIG_BLUFI_ENABLE
}

static device_handle_t agora_device_bringup(sys_up_mode_e mode)
{
  char *ssid    = NULL;
  char *psw     = NULL;
  char *license = NULL;
  char *product_key = NULL;

  // 1. load device state config
  device_handle_t dev_state = load_devic_state();

  // 2. network config and connect
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

    setup_wifi();

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
  }

  if (0 != device_get_item_string(dev_state, "product_key", &product_key)) {
    ESP_LOGE(TAG, "cannot found product_key in device state items\n");
    goto dev_bringup_err;
  }

  // 4. activate the device
  char user_account[64] = { 0 };
  if (SYS_UP_MODE_WAKEUP != mode) {
    // maybe nee to avtiate device if was not wakeup frome low-power mode
    if (SYS_UP_MODE_RESTORE == mode || 0 != agora_iot_query_user(CONFIG_MASTER_SERVER_URL, product_key, get_device_id(), user_account, AGORA_IOT_CLIENT_ID_MAX_LEN) ||
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
  char *user_id = NULL;

  if (0 != device_get_item_string(dev_state, "dev_crt", &dev_crt) ||
      0 != device_get_item_string(dev_state, "dev_key", &dev_key) ||
      0 != device_get_item_string(dev_state, "domain", &domain)   ||
      0 != device_get_item_string(dev_state, "client_id", &client_id) ||
      0 != device_get_item_string(dev_state, "bind_user", &user_id)) {
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
    .user_id     = user_id,
    .enable_rtc  = true,
    .certificate = license,
    .enable_recv_audio = true,
    .enable_recv_video = false,
    .area_code         = AGO_AREA_CODE_GLOB,
    .rtc_cb = {
      .cb_start_push_frame    = iot_cb_start_push_frame,
      .cb_stop_push_frame     = iot_cb_stop_push_frame,
      .cb_receive_audio_frame = iot_cb_receive_audio_frame,
      .cb_receive_video_frame = iot_cb_receive_video_frame,
      .cb_target_bitrate_changed = iot_cb_target_bitrate_changed,
      .cb_key_frame_requested    = iot_cb_key_frame_requested,
    },
    .disable_rtc_log      = true,
    .log_level            = AGORA_LOG_INFO,
    .max_possible_bitrate = BWE_MAX_BPS,
    .enable_audio_config  = true,
    .audio_config = {
      .audio_codec_type = AUDIO_CODEC_TYPE,
#if defined(CONFIG_SEND_PCM_DATA)
      .pcm_sample_rate  = CONFIG_PCM_SAMPLE_RATE,
      .pcm_channel_num  = I2S_CHANNELS,
#endif
    },

    .slave_server_url = CONFIG_SLAVE_SERVER_URL,
    .call_mode        = CALL_MODE_MUTLI,
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


#if (LOWER_POWER_MODE == CONFIG_ENABLE_DEEP_SLEEP)
#define CONFIG_ENTER_DEEPSLEEP_THRESHOLD 20

#define CONFIG_EXAMPLE_EXT0_WAKEUP    1
#define ESP_EXT0_WAKEUP_LEVEL_LOW 0
#define ESP_EXT0_WAKEUP_LEVEL_HIGH 1

static uint8_t g_idle_loop_count = 0;
static int wakeup_ext = 0;

static RTC_DATA_ATTR struct timeval sleep_enter_time;
static void handle_wakeup_cause(void)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
#if CONFIG_EXAMPLE_EXT0_WAKEUP
        case ESP_SLEEP_WAKEUP_EXT0: {
            printf("[%d] Wake up from ext0\n", xTaskGetTickCount());
            wakeup_ext = 1;
            break;
        }
#endif // CONFIG_EXAMPLE_EXT0_WAKEUP
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("[%d] Not a deep sleep reset %d\n", xTaskGetTickCount(), esp_sleep_get_wakeup_cause());
            break;
    }
}

static void deep_sleep_set_and_start(void)
{
#ifdef CONFIG_EXAMPLE_EXT0_WAKEUP
    const int ext_wakeup_pin_0 = GPIO_NUM_5;
    ESP_ERROR_CHECK(rtc_gpio_init(ext_wakeup_pin_0));
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(ext_wakeup_pin_0, ESP_EXT0_WAKEUP_LEVEL_LOW));
    ESP_LOGW(TAG, "Enabling ext0 wakeup on pins GPIO%d\n", ext_wakeup_pin_0);  
    ESP_ERROR_CHECK(gpio_pullup_en(ext_wakeup_pin_0));
    ESP_ERROR_CHECK(gpio_pulldown_dis(ext_wakeup_pin_0));  
#endif // CONFIG_EXAMPLE_EXT0_WAKEUP

    gettimeofday(&sleep_enter_time, NULL);
    // esp_sleep_enable_timer_wakeup(20000000);

    printf("Entering deep sleep\r\n");
    esp_deep_sleep_start();
}
#endif


int app_main(void)
{
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

#if (LOWER_POWER_MODE == CONFIG_ENABLE_LIGHT_SLEEP)
  esp_pm_config_esp32s3_t pm_config = { .max_freq_mhz = CONFIG_EXAMPLE_MAX_CPU_FREQ_MHZ,
                                        .min_freq_mhz = CONFIG_EXAMPLE_MIN_CPU_FREQ_MHZ,
                                        .light_sleep_enable = true };
  ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#elif (LOWER_POWER_MODE == CONFIG_ENABLE_DEEP_SLEEP)
    /* deep sleep mode */
    handle_wakeup_cause();
#endif

#ifdef CONFIG_ENABLE_RUN_TIME_STATS
  esp_timer_create_args_t create_args = { .callback = fps_timer_callback, .arg = NULL, .name = "fps timer" };
  esp_timer_create(&create_args, &fps_timer);
  esp_timer_start_periodic(fps_timer, 20 * 1000 * 1000);
  tick_begin = xTaskGetTickCount();
  image_cnt  = 0;
#endif

  // Initialize peripherals management
  ESP_LOGI(TAG, "Initialize peripherals");
  esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
  esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

#ifdef CONFIG_SDCARD
    ESP_LOGI(TAG, "[1.0] Mount sdcard");
    // Initialize SD Card peripheral
    audio_board_sdcard_init(set, SD_MODE_1_LINE);
#endif

  // Monitor the key event
  start_key_service(set);

#ifndef CONFIG_AUDIO_ONLY
#ifdef UVC_STREAM_ENABLE
  setup_uvc_stream();
#else //#ifdef UVC_STREAM_ENABLE
  init_camera();

  jpg_encoder = init_jpeg_encoder(40, 0, 20, JPEG_SUB_SAMPLE_YUV420);
  if (!jpg_encoder) {
    ESP_LOGE(TAG, "Failed to initialize jpeg encoder!");
    goto EXIT;
  }
#endif //#ifdef UVC_STREAM_ENABLE
#endif

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

  setup_audio();

  create_capture_task();

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
#if (LOWER_POWER_MODE == CONFIG_ENABLE_DEEP_SLEEP)
      g_idle_loop_count = 0;
#endif
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);

#if (LOWER_POWER_MODE == CONFIG_ENABLE_DEEP_SLEEP)
    if (g_idle_loop_count++ < CONFIG_ENTER_DEEPSLEEP_THRESHOLD) {
        continue;
    }

    deep_sleep_set_and_start();
#endif
  }

EXIT:
  // Deinit Agora IoT SDK
  if (g_handle) { 
    agora_iot_deinit(g_handle);
  }

  if (dev_state) {
    device_destroy_state(dev_state);
  }

#ifdef UVC_STREAM_ENABLE
  uvc_streaming_stop();
#endif

#ifdef CONFIG_ENABLE_RUN_TIME_STATS
  esp_timer_stop(fps_timer);
  esp_timer_delete(fps_timer);
#endif

  ESP_LOGW(TAG, "App exited.");
  return 0;
}
