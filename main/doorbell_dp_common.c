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

#define STATE_FILE_SIZE_MAX (1024 * 16)

#define STORAGE_NAMESPACE "storage"

static char g_node_id[32] = { 0 };
static char g_node_secret[64] = { 0 };

const char *get_node_id(void)
{
  // TODO: should get device from system at here
  if (g_node_id[0] == 0) {
    strncpy(g_node_id, CONFIG_NODE_ID, sizeof(g_node_id));
  }
  return g_node_id;
}

const char *get_node_secret(void)
{
  if (g_node_secret[0] == 0) {
    strncpy(g_node_secret, CONFIG_NODE_SECRET, sizeof(g_node_secret));
  }
  return g_node_secret;
}

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
  const char *node_id = get_node_id();
  const char *node_secret = get_node_secret();

  // 1. activate license
  char activate_res[1024] = { 0 };
  if (0 != agora_iot_node_activate(CONFIG_AGORA_APP_ID, node_id, node_secret, activate_res, sizeof(activate_res))) {
    printf("cannot activate agora license !\n");
    goto activate_err;
  }
  device_set_item_string(dev_state, "activate_res", activate_res);

  return 0;

activate_err:
  return -1;
}
