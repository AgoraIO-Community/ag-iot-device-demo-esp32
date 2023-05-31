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

#ifndef __AGORA_IOT_FILE_PLAYER_H__
#define __AGORA_IOT_FILE_PLAYER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "agora_iot_api.h"

typedef void* file_player_handle_t;

typedef struct agora_iot_file_player_callback
{
  /**
   * @brief Occurs when sdk is ready to send video/audio frame.
   * This callback notifies the sender to send video/audio frame.
   */
  void (*cb_start_push_frame)(const char *channel_name);

  /**
   * @brief Occurs when sdk stops sending video/audio frame.
   * This callback notifies the sender to stop sending video/audio frame.
   */
  void (*cb_stop_push_frame)(const char *channel_name);
} agora_iot_file_player_callback_t;

/**
 * @brief global file player initialize
 * 
 * @param slave_server_url IoT Call Server
 * @param appid appid, use macro CONFIG_AGORA_APP_ID
 * @return int 
 * - = 0: success
 * - < 0: failure
 */
int agora_iot_file_player_init(char *slave_server_url, const char *appid);

/**
 * @brief global file player destroy, release all resource
 * 
 * @return void 
 */
void agora_iot_file_player_deinit(void);

/**
 * @brief start file player
 * 
 * @param callback      will be ccalled when need to start or stop push frame
 * @param channel_name  the channel used for transfer data with receive node
 * @param audio_config  audio codec info that you want to use for transfer
 * @return file_player_handle_t 
 */
file_player_handle_t agora_iot_file_player_start(agora_iot_file_player_callback_t callback,
                                                 const char *channel_name, agora_iot_audio_config_t *audio_config);

/**
 * @brief stop file player
 * 
 * @param handle  handle of a player object, from agora_iot_file_player_start
 * @return int 
 * - = 0: success
 * - < 0: failure
 */
int agora_iot_file_player_stop(file_player_handle_t handle);

/**
 * @brief push video frame when cb_start_push_frame trigger
 * 
 * @param handle  handle of a player object, from agora_iot_file_player_start
 * @param frame   video frame info
 * @return int 
 * - = 0: success
 * - < 0: failure
 */
int agora_iot_file_player_push_video_frame(file_player_handle_t handle, ago_video_frame_t *frame);

/**
 * @brief push audio frame when cb_start_push_frame trigger
 * 
 * @param handle  handle of a player object, from agora_iot_file_player_start
 * @param frame   audio frame info, must be 20ms length if type is PCM
 * @return int 
 * - = 0: success
 * - < 0: failure
 */
int agora_iot_file_player_push_audio_frame(file_player_handle_t handle, ago_audio_frame_t *frame);

#ifdef __cplusplus
}
#endif
#endif // __AGORA_IOT_FILE_PLAYER_H__