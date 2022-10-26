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

typedef void* file_player_handle_t;

typedef enum {
  FILE_VIDEO_H264 = 0,
  FILE_VIDEO_JPEG,
} agora_iot_file_video_e;

typedef struct {
  uint8_t  *frame;
  size_t   frame_len;
  uint32_t codec; /* agora_iot_file_video_e */
  bool     keyframe;
  uint32_t frame_rate;
} agora_iot_file_video_t;

typedef struct {
  uint8_t  *frame; /* must be PCM(16K 1channel 16bit, 20ms for one frame) */
  size_t   frame_len; /* must be 20ms, 640byte */
} agora_iot_file_audio_t;

typedef struct agora_iot_file_player_callback
{
  /**
   * @brief Occurs when sdk is ready to send video/audio frame.
   * This callback notifies the sender to send video/audio frame.
   */
  void (*cb_start_push_frame)(void);

  /**
   * @brief Occurs when sdk stops sending video/audio frame.
   * This callback notifies the sender to stop sending video/audio frame.
   */
  void (*cb_stop_push_frame)(void);
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
 * @param callback 
 * @param channel_name 
 * @return file_player_handle_t 
 */
file_player_handle_t agora_iot_file_player_start(agora_iot_file_player_callback_t callback,
                                                 const char *channel_name);

/**
 * @brief stop file player
 * 
 * @param handle 
 * @return int 
 * - = 0: success
 * - < 0: failure
 */
int agora_iot_file_player_stop(file_player_handle_t handle);

/**
 * @brief push video frame when cb_start_push_frame trigger
 * 
 * @param handle 
 * @param frame 
 * @return int 
 * - = 0: success
 * - < 0: failure
 */
int agora_iot_file_player_push_video_frame(file_player_handle_t handle, agora_iot_file_video_t *frame);

/**
 * @brief push audio frame when cb_start_push_frame trigger
 * 
 * @param handle 
 * @param frame 
 * @return int 
 * - = 0: success
 * - < 0: failure
 */
int agora_iot_file_player_push_audio_frame(file_player_handle_t handle, agora_iot_file_audio_t *frame);

#ifdef __cplusplus
}
#endif
#endif // __AGORA_IOT_FILE_PLAYER_H__