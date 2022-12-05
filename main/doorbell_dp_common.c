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

#include <stdio.h>
#include <unistd.h>

#include <string.h>
#include <malloc.h>

#include <time.h>
#include <sys/time.h>

#include "agora_iot_api.h"
#include "agora_iot_call.h"
#include "agora_iot_dp.h"
#include "agora_iot_device_manager.h"

#include "app_config.h"

#include "device_state.h"
#include "doorbell_dp_common.h"

#include "esp_partition.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "audio_mem.h"

#include "nvs.h"

#define TAG "DOORBELL_DP"

static char g_device_id[32] = { 0 };

#define STATE_FILE_SIZE_MAX (1024 * 16)

/* Data Point ID start */
/* AGORA_DP_ID_<TYPE>_<NAME> */
#define AGORA_DP_ID_BOOL_OSD_WATERMARK_SWITCH       100
#define AGORA_DP_ID_ENUM_INFRARED_NIGHT_VISION      101
#define AGORA_DP_ID_BOOL_MOVING_WARNING             102
#define AGORA_DP_ID_ENUM_PIR_SWITCH                 103
#define AGORA_DP_ID_INT_VOLUME_CONTROL              104
#define AGORA_DP_ID_BOOL_FORCE_RELEASING_WARNING    105
#define AGORA_DP_ID_INT_BATTERY_CAPACITY            106
#define AGORA_DP_ID_ENUM_VIDEO_ART                  107
#define AGORA_DP_ID_BOOL_WORK_LED                   108
#define AGORA_DP_ID_STR_FIRMWARE_VER                109
#define AGORA_DP_ID_ENUM_TF_STATE                   110
#define AGORA_DP_ID_INT_TF_STORAGE_AVAILABLE        111
#define AGORA_DP_ID_ENUM_TF_FORMAT_CTRL             112
#define AGORA_DP_ID_INT_PREVIEW_TIME                113
#define AGORA_DP_ID_BOOL_ALRAM_SONG                 114
#define AGORA_DP_ID_BOOL_VOICE_SENSE                115

#define AGORA_DP_ID_STR_WIFI_SSID                   501
#define AGORA_DP_ID_STR_DEVICE_IP                   502
#define AGORA_DP_ID_STR_DEVICE_MAC                  503
#define AGORA_DP_ID_STR_TIME_ZONE                   504

#define AGORA_DP_ID_ENUM_STANDBY_STATE              1000
/* Data Point ID end */

#define STR_LEN 32
static char g_str_firmware_ver[STR_LEN] = "ver.1.0.0.3";

/* The mode of the data point is decided by the Application */
typedef enum doorbell_dp_info_mode {
  MODE_R, // read only
  MODE_RW // read & write
} doorbell_dp_info_mode_e;
typedef struct doorbell_dp_info {
  doorbell_dp_info_mode_e mode;
  agora_dp_info_t info;
} doorbell_dp_info_t;

static doorbell_dp_info_t g_mock_dp_state[] = {
  /* DP ID: 100 ~ 1** */
  { .mode = MODE_RW, .info.dp_id = AGORA_DP_ID_BOOL_OSD_WATERMARK_SWITCH, .info.dp_type = AGORA_DP_TYPE_BOOL, .info.dp_value.dp_bool = false, },
  { .mode = MODE_RW, .info.dp_id = AGORA_DP_ID_ENUM_INFRARED_NIGHT_VISION, .info.dp_type = AGORA_DP_TYPE_ENUM, .info.dp_value.dp_enum = 0, },
  { .mode = MODE_RW, .info.dp_id = AGORA_DP_ID_BOOL_MOVING_WARNING, .info.dp_type = AGORA_DP_TYPE_BOOL, .info.dp_value.dp_bool = false, },
  { .mode = MODE_RW, .info.dp_id = AGORA_DP_ID_ENUM_PIR_SWITCH, .info.dp_type = AGORA_DP_TYPE_ENUM, .info.dp_value.dp_enum = 0, },
  { .mode = MODE_RW, .info.dp_id = AGORA_DP_ID_INT_VOLUME_CONTROL, .info.dp_type = AGORA_DP_TYPE_INT, .info.dp_value.dp_int = 50, },
  { .mode = MODE_RW, .info.dp_id = AGORA_DP_ID_BOOL_FORCE_RELEASING_WARNING, .info.dp_type = AGORA_DP_TYPE_BOOL, .info.dp_value.dp_bool = false, },
  { .mode = MODE_R,  .info.dp_id = AGORA_DP_ID_INT_BATTERY_CAPACITY, .info.dp_type = AGORA_DP_TYPE_INT, .info.dp_value.dp_int = 50, },
  { .mode = MODE_RW, .info.dp_id = AGORA_DP_ID_ENUM_VIDEO_ART, .info.dp_type = AGORA_DP_TYPE_ENUM, .info.dp_value.dp_enum = 1, },
  { .mode = MODE_RW, .info.dp_id = AGORA_DP_ID_BOOL_WORK_LED, .info.dp_type = AGORA_DP_TYPE_BOOL, .info.dp_value.dp_bool = false, },
  { .mode = MODE_R,  .info.dp_id = AGORA_DP_ID_STR_FIRMWARE_VER, .info.dp_type = AGORA_DP_TYPE_STR, .info.dp_value.dp_str = g_str_firmware_ver},
  { .mode = MODE_R,  .info.dp_id = AGORA_DP_ID_ENUM_TF_STATE, .info.dp_type = AGORA_DP_TYPE_ENUM, .info.dp_value.dp_enum = 1},
  { .mode = MODE_R,  .info.dp_id = AGORA_DP_ID_INT_TF_STORAGE_AVAILABLE, .info.dp_type = AGORA_DP_TYPE_INT, .info.dp_value.dp_int = 1024},
  { .mode = MODE_RW, .info.dp_id = AGORA_DP_ID_ENUM_TF_FORMAT_CTRL, .info.dp_type = AGORA_DP_TYPE_ENUM, .info.dp_value.dp_enum = 1},
  { .mode = MODE_RW, .info.dp_id = AGORA_DP_ID_INT_PREVIEW_TIME, .info.dp_type = AGORA_DP_TYPE_INT, .info.dp_value.dp_int = 60},
  { .mode = MODE_RW, .info.dp_id = AGORA_DP_ID_BOOL_ALRAM_SONG, .info.dp_type = AGORA_DP_TYPE_BOOL, .info.dp_value.dp_bool = false, },
  { .mode = MODE_RW, .info.dp_id = AGORA_DP_ID_BOOL_VOICE_SENSE, .info.dp_type = AGORA_DP_TYPE_BOOL, .info.dp_value.dp_bool = false, },
  /* DP ID: 500 ~  */
  // { .mode = MODE_R,  .info.dp_id = AGORA_DP_ID_STR_WIFI_SSID, .info.dp_type = AGORA_DP_TYPE_STR, .info.dp_value.dp_str = g_str_wifi_ssid},
  // { .mode = MODE_R,  .info.dp_id = AGORA_DP_ID_STR_DEVICE_IP, .info.dp_type = AGORA_DP_TYPE_STR, .info.dp_value.dp_str = g_str_device_ip},
  // { .mode = MODE_R,  .info.dp_id = AGORA_DP_ID_STR_DEVICE_MAC, .info.dp_type = AGORA_DP_TYPE_STR, .info.dp_value.dp_str = g_str_device_mac},
  // { .mode = MODE_R,  .info.dp_id = AGORA_DP_ID_STR_TIME_ZONE, .info.dp_type = AGORA_DP_TYPE_STR, .info.dp_value.dp_str = g_str_time_zone},
  { .mode = MODE_RW, .info.dp_id = AGORA_DP_ID_ENUM_STANDBY_STATE, .info.dp_type = AGORA_DP_TYPE_ENUM, .info.dp_value.dp_int = 2, },
};
static int g_mock_dp_state_total = sizeof(g_mock_dp_state) / sizeof(doorbell_dp_info_t);

static void _query_callback(agora_dp_info_t *info, void *args)
{
  ESP_LOGI(TAG, "------query DP[%d], type[%d]", info->dp_id, info->dp_type);

  for (int i = 0; i < g_mock_dp_state_total; i++) {
    if (info->dp_id == g_mock_dp_state[i].info.dp_id) {
      if (info->dp_type == g_mock_dp_state[i].info.dp_type) {
        info->dp_value = g_mock_dp_state[i].info.dp_value;

        switch (g_mock_dp_state[i].info.dp_type) {
        case AGORA_DP_TYPE_INT:
          ESP_LOGI(TAG, ", value[%d]\n", info->dp_value.dp_int);
          return;
        case AGORA_DP_TYPE_BOOL:
          ESP_LOGI(TAG, ", value[%s]\n", info->dp_value.dp_bool ? "true" : "false");
          return;
        case AGORA_DP_TYPE_ENUM:
          ESP_LOGI(TAG, ", value[%d]\n", info->dp_value.dp_enum);
          return;
        case AGORA_DP_TYPE_STR:
          ESP_LOGI(TAG, ", value[%s]\n", info->dp_value.dp_str ? info->dp_value.dp_str : "null");
          return;
        default:
          /* This case should not be ran */
          ESP_LOGI(TAG, "\n");
          return;
        }
      } else {
        ESP_LOGE(TAG, "\n------and the type does't match to the type[%d]\n", g_mock_dp_state[i].info.dp_type);
        return;
      }
    }
  }

  ESP_LOGE(TAG, "\n------and cann't find the DP");
}

static void _cmd_callback(const agora_dp_info_t *info, void *args)
{
  ESP_LOGI(TAG, "------control DP[%d], type[%d]", info->dp_id, info->dp_type);

  for (int i = 0; i < g_mock_dp_state_total; i++) {
    if (info->dp_id == g_mock_dp_state[i].info.dp_id) {
      if (MODE_RW != g_mock_dp_state[i].mode) {
        ESP_LOGE(TAG, "\n------the data point is read only\n");
        return;
      }

      if (info->dp_type == g_mock_dp_state[i].info.dp_type) {
        switch (g_mock_dp_state[i].info.dp_type) {
        case AGORA_DP_TYPE_INT:
          g_mock_dp_state[i].info.dp_value.dp_int = info->dp_value.dp_int;
          ESP_LOGI(TAG, ", value[%d]\n", g_mock_dp_state[i].info.dp_value.dp_int);
          return;
        case AGORA_DP_TYPE_BOOL:
          g_mock_dp_state[i].info.dp_value.dp_bool = info->dp_value.dp_bool;
          ESP_LOGI(TAG, ", value[%s]\n", g_mock_dp_state[i].info.dp_value.dp_bool ? "true" : "false");
          return;
        case AGORA_DP_TYPE_ENUM:
          g_mock_dp_state[i].info.dp_value.dp_enum = info->dp_value.dp_enum;
          ESP_LOGI(TAG, ", value[%d]\n", g_mock_dp_state[i].info.dp_value.dp_enum);
          return;
        case AGORA_DP_TYPE_STR:
          memset(g_mock_dp_state[i].info.dp_value.dp_str, 0, STR_LEN);
          snprintf(g_mock_dp_state[i].info.dp_value.dp_str, STR_LEN, "%s", info->dp_value.dp_str);
          ESP_LOGI(TAG, ", value[%s]\n", g_mock_dp_state[i].info.dp_value.dp_str ?
                  g_mock_dp_state[i].info.dp_value.dp_str : "null");
          return;
        default:
          /* This case should not be ran */
          ESP_LOGI(TAG, "\n");
          return;
        }
      } else {
        ESP_LOGE(TAG, "\n------and the type does't match to the type[%d]\n", g_mock_dp_state[i].info.dp_type);
        return;
      }
    }
  }

  ESP_LOGE(TAG, "\n------and cann't find the DP");
}

char *get_device_id(void)
{
  uint8_t base_mac_addr[6] = {0};

  ESP_ERROR_CHECK(esp_efuse_mac_get_default(base_mac_addr));

  uint8_t tmp = 0x00;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 2; j++) {
      tmp = (*(base_mac_addr + i) >> 4) * (1 - j) + (*(base_mac_addr + i) & 0x0F) * j;
      if (tmp >= 0 && tmp <= 9) {
        g_device_id[2 * i + j] = tmp + '0';
      } else if (tmp >= 0x0A && tmp <= 0x0F) {
        g_device_id[2 * i + j] = tmp - 0x0A + 'A';
      }
    }
  }

  return g_device_id;
}

#define STORAGE_NAMESPACE "storage"

device_handle_t load_devic_state(void)
{
  device_handle_t dev_state = NULL;
  char* file_buf = NULL;

#ifdef NVS_STORAGE
  nvs_handle_t my_handle;
  esp_err_t err;

  // Open
  err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
  if (err != ESP_OK)
    goto load_state_err;

  // Read blob
  size_t required_size = 0;
  // obtain required memory space to store blob being read from NVS
  err = nvs_get_blob(my_handle, "dev_state", NULL, &required_size);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    goto load_state_err;

  if (required_size == 0) {
    ESP_LOGE(TAG, "Nothing saved yet!\n");
    goto load_state_err;
  } else {
    file_buf = (char *)malloc(required_size);
    err = nvs_get_blob(my_handle, "dev_state", file_buf, &required_size);
    if (err != ESP_OK) {
      goto load_state_err;
    }
  }
#else
  // malloc buffer to read file
  file_buf = (char *)malloc(STATE_FILE_SIZE_MAX);
  if (NULL == file_buf) {
    ESP_LOGE(TAG, "malloc failed !\n");
    goto load_state_err;
  }
  memset(file_buf, 0, STATE_FILE_SIZE_MAX);

  // Find the partition map in the partition table
  const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
  if (NULL == partition) {
    ESP_LOGE(TAG, "load_devic_state: partition find failed !\n");
    goto load_state_err;
  }


  ESP_ERROR_CHECK(esp_partition_read(partition, 0, file_buf, STATE_FILE_SIZE_MAX));
#endif

  dev_state = device_load_state(file_buf);
  if (NULL == dev_state) {
    ESP_LOGE(TAG, "dev_state is NULL !\n");
    goto load_state_err;
  }

load_state_err:
  if (file_buf) {
    free(file_buf);
  }

#ifdef NVS_STORAGE
  // Close
  nvs_close(my_handle);
#endif
  return dev_state;
}

int save_device_state(device_handle_t dev_state)
{
#ifdef NVS_STORAGE
  nvs_handle_t my_handle;
  esp_err_t err;

  // Open
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
    return err;  

  char *state_context = (char *)device_build_state_content(dev_state);
  if (NULL == state_context) {
    ESP_LOGE(TAG, "device_build_state_content failed !\n");
    return -1;
  } 

  err = nvs_set_blob(my_handle, "dev_state", state_context, strlen(state_context));
  if (err != ESP_OK)
    return err;

  // Commit
  err = nvs_commit(my_handle);
  if (err != ESP_OK)
    return err;

  // Close
  nvs_close(my_handle);
#else
  // Find the partition map in the partition table
  const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
  if (NULL == partition) {
    ESP_LOGE(TAG, "save_device_state: partition find failed !\n");
    return -1;
  }

  ESP_ERROR_CHECK(esp_partition_erase_range(partition, 0, partition->size));

  char *state_context = (char *)device_build_state_content(dev_state);
  if (NULL == state_context) {
    ESP_LOGE(TAG, "device_build_state_content failed !\n");
    return -1;
  }

  // Write the data, starting from the beginning of the partition
  ESP_ERROR_CHECK(esp_partition_write(partition, 0, state_context, strlen(state_context)));
#endif
  free(state_context);
  return 0;
}


int activate_device(device_handle_t dev_state)
{
  int ret           = -1;
  char *user_id     = NULL;
  char *device_name = NULL;
  char *product_key = NULL;
  char *device_id   = NULL;
  char *cert        = NULL;

  if (0 != device_get_item_string(dev_state, "product_key", &product_key)) {
    ESP_LOGE(TAG, "cannot found product_key in device state items\n");
    goto activate_err;
  }

  device_id = get_device_id();

  // 1. activate license
  if (0 != agora_iot_license_activate(CONFIG_AGORA_APP_ID, CONFIG_CUSTOMER_KEY, CONFIG_CUSTOMER_SECRET,
                                      CONFIG_PRODUCT_KEY, device_id, CONFIG_LICENSE_PID, &cert)) {
    ESP_LOGE(TAG, "cannot activate agora license !\n");
    goto activate_err;
  }
  device_set_item_string(dev_state, "license", cert);

  // 2. register DP service
  if (0 != device_get_item_string(dev_state, "user_id", &user_id)) {
    ESP_LOGE(TAG, "cannot found user_id from device state.\n");
    goto activate_err;
  }
  if (0 != device_get_item_string(dev_state, "device_name", &device_name)) {
    ESP_LOGE(TAG, "cannot found device_name from device state.\n");
    goto activate_err;
  }

  printf("agora_iot_register_and_bind: product_key %s, device_id %s, user_id %s, device_name %s\n",
          product_key, device_id, user_id, device_name);

  agora_iot_device_info_t device_info = { 0 };
  if (0 != agora_iot_register_and_bind(CONFIG_MASTER_SERVER_URL, product_key, device_id,
                                      user_id, device_name, &device_info)) {
    ESP_LOGE(TAG, "register device to aws failure\n");
    goto activate_err;
  }

  device_set_item_string(dev_state, "domain", device_info.domain);
  device_set_item_string(dev_state, "dev_crt", device_info.certificate);
  device_set_item_string(dev_state, "dev_key", device_info.private_key);
  device_set_item_string(dev_state, "client_id", device_info.client_id);

  // 3. get bind user info
  char user_account[64] = { 0 };
  int rval = agora_iot_query_user(CONFIG_MASTER_SERVER_URL, product_key, device_id, user_account, AGORA_IOT_CLIENT_ID_MAX_LEN);
  if (0 != rval) {
    ESP_LOGE(TAG, "query device manager user failed\n");
    goto activate_err;
  }
  device_set_item_string(dev_state, "bind_user", user_account);

  ret = 0;

activate_err:
  if (cert) {
    free(cert);
  }
  if (user_id) {
    free(user_id);
  }
  if (device_name) {
    free(device_name);
  }
  if (product_key) {
    free(product_key);
  }
  return ret;
}

int update_device_work_state(agora_iot_handle_t handle, sys_up_mode_e mode)
{
  /* Register the query callback for every data point, and only register the command callback for some data points */
  for (int i = 0; i < g_mock_dp_state_total; i++) {
    agora_iot_dp_register_dp_query_handler(handle, g_mock_dp_state[i].info.dp_id, g_mock_dp_state[i].info.dp_type,
                                            _query_callback, (void *)handle);

    if (MODE_RW == g_mock_dp_state[i].mode) {
      agora_iot_dp_register_dp_cmd_handler(handle, g_mock_dp_state[i].info.dp_id, g_mock_dp_state[i].info.dp_type,
                                            _cmd_callback, (void *)handle);
    }
  }

  /* have to update at last once to create shadow, do it in SDK */

  if (SYS_UP_MODE_WAKEUP != mode) {
    agora_iot_dp_publish_all(handle);
  } else {
    agora_dp_info_t dp_info = { 0 };
    dp_info.dp_id           = AGORA_DP_ID_ENUM_STANDBY_STATE;
    dp_info.dp_type         = AGORA_DP_TYPE_ENUM;
    dp_info.dp_value.dp_int = 2;
    agora_iot_dp_publish(handle, &dp_info);
  }

  return 0;
}

void update_device_low_power(agora_iot_handle_t handle)
{
  agora_dp_info_t dp_info = { 0 };
  dp_info.dp_id           = AGORA_DP_ID_ENUM_STANDBY_STATE;
  dp_info.dp_type         = AGORA_DP_TYPE_ENUM;
  dp_info.dp_value.dp_int = 1;
  agora_iot_dp_publish(handle, &dp_info);
}

int start_alarm_record(agora_iot_handle_t handle, unsigned long long begin_time)
{
  int ret = agora_iot_cloud_record_start(handle, begin_time, SEND_AUDIO_DATA_TYPE, SEND_VIDEO_DATA_TYPE);
  if (0 != ret) {
    printf("#### start recording failure: %d\n", ret);
    return ret;
  }

  return 0;
}

int stop_alarm_record(agora_iot_handle_t handle)
{
  struct timeval tv;
  if (gettimeofday (&tv, NULL) < 0) {
    printf("#### query system time failure\n");
    return -1;
  }

  unsigned long long end_time = (unsigned long long)((uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000);
  return agora_iot_cloud_record_stop(handle, end_time);
}

int alarm_message_send(agora_iot_handle_t handle, unsigned long long begin_time, agora_iot_file_info_t file_info, char *nick_name, agora_iot_alarm_type_e alarm_type, char *alarm_desc)
{
  int ret = 0;
  char *image_id = NULL;

  if (file_info.buf != NULL) {
    ret = agora_iot_push_alarm_image(handle, &file_info, &image_id);
    if (ret < 0) {
      printf("#### agora_iot_push_alarm_image failure %d\n", ret);
      return ret;
    }
  } else {
    image_id = audio_calloc(1, 32);
    strncpy(image_id, "alarm_message_rand", 32);
  }

  ret = agora_iot_push_alarm_message(handle, begin_time, nick_name, alarm_type, alarm_desc, image_id);
  if (0 != ret) {
    printf("#### alarm failure: %d\n", ret);
    return ret;
  }

  if (image_id) {
    free(image_id);
  }

  return 0;
}