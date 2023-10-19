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

#ifndef __AGORA_IOT_API_H__
#define __AGORA_IOT_API_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Error Code define
typedef enum {
  /* No Error */
  AGORA_ERR_SUCCESS = 0,

  /* General error (no specified reason). */
  AGORA_ERR_FAILED = -1,

  /* Invalid argument called */
  AGORA_ERR_INVALID_ARGUMENT = -2,

  /* The required component is not enabled */
  AGORA_ERR_COMPONENT_NOT_ENABLED = -3,

  /* The feature is disabled */
  AGORA_ERR_FEATURE_DISABLED = -4
} agora_iot_error_e;

/**
 * @brief The data types which the SDK supports, include video and audio.
 * Send the frames with the below types.
 */
typedef enum {
  /**
   * 0: YUV420
   */
  AGORA_VIDEO_DATA_TYPE_YUV420 = 0,

  /**
   * 1: H264
   */
  AGORA_VIDEO_DATA_TYPE_H264 = 1,

  /**
   * 2: JPEG
   */
  AGORA_VIDEO_DATA_TYPE_JPEG = 2,

  /**
   * 3: H265
   */
  AGORA_VIDEO_DATA_TYPE_H265 = 3,

  /**
   * 10: PCM
   */
  AGORA_AUDIO_DATA_TYPE_PCM = 10,

  /**
   * 11: OPUS
   * support SampleRate: 16000HZ; Bits: 16; Channel: 1
   */
  AGORA_AUDIO_DATA_TYPE_OPUS = 11,

  /**
   * 12: PCMA
   * support SampleRate: 8000HZ; Bits: 16; Channel: 1
   */
  AGORA_AUDIO_DATA_TYPE_G711A = 12,

  /**
   * 13: PCMU
   * support SampleRate: 8000HZ; Bits: 16; Channel: 1
   */
  AGORA_AUDIO_DATA_TYPE_G711U = 13,

  /**
   * 14: G722
   * support SampleRate: 16000HZ; Bits: 16; Channel: 1
   */
  AGORA_AUDIO_DATA_TYPE_G722 = 14,

  /**
   * 15: AACLC
   */
  AGORA_AUDIO_DATA_TYPE_AACLC = 15,

  /**
   * 16: HEAAC
   */
  AGORA_AUDIO_DATA_TYPE_HEAAC = 16,
} agora_data_type_e;

typedef struct {
  /* vidoe frame data type */
  agora_data_type_e data_type;
  /* keyframe or not */
  bool is_key_frame;
  /* frame data buffer, SSP & PPS must be carried with keyframe data*/
  uint8_t *video_buffer;
  /* frame data buffer size */
  uint32_t video_buffer_size;
  /* frames per second, must be accurate for cloud record */
  uint8_t fps;
} agora_video_frame_t;

typedef struct {
  /* audio frame data type */
  agora_data_type_e data_type;
  /* frame data buffer, must be 20ms data in one frame */
  uint8_t *audio_buffer;
  /* frame data buffer size */
  uint32_t audio_buffer_size;
} agora_audio_frame_t;

/**
 * Go fitting video server areas.
 */
typedef enum {
  /* the same as AREA_CODE_GLOB */
  AGORA_AREA_CODE_DEFAULT = 0x00000000,
  /* Mainland China. */
  AGORA_AREA_CODE_CN = 0x00000001,
  /* North America. */
  AGORA_AREA_CODE_NA = 0x00000002,
  /* Europe. */
  AGORA_AREA_CODE_EU = 0x00000004,
  /* Asia, excluding Mainland China. */
  AGORA_AREA_CODE_AS = 0x00000008,
  /* Japan. */
  AGORA_AREA_CODE_JP = 0x00000010,
  /* India. */
  AGORA_AREA_CODE_IN = 0x00000020,
  /* Oceania */
  AGORA_AREA_CODE_OC = 0x00000040,
  /* South-American */
  AGORA_AREA_CODE_SA = 0x00000080,
  /* Africa */
  AGORA_AREA_CODE_AF = 0x00000100,
  /* South Korea */
  AGORA_AREA_CODE_KR = 0x00000200,
  /* The global area (except China) */
  AGORA_AREA_CODE_OVS = 0xFFFFFFFE,
  /* (Default) Global. */
  AGORA_AREA_CODE_GLOB = (0xFFFFFFFF),
} agora_areas_type_e;

/**
 * Status code for server connection.
 */
typedef enum {
  /** connected */
  AGORA_IOT_CONNECTED = 0,
  /** reconnecting */
  AGORA_IOT_RECONNECTING,
  /** disconnected */
  AGORA_IOT_DISCONNECTED,
} agora_iot_connection_status_e;

/**
 * @brief Agora IOT SDK Event Callback
 */
typedef struct agora_iot_event_handler {
  /**
   * Report error message during runtime.
   *
   * In most cases, it means SDK can't fix the issue and application should take action.
   *
   * @param[in] code    Error code, see #agora_err_code_e
   * @param[in] msg     Error message
   *
   */
  void (*on_error)(int code, const char *msg);

  /**
   * @brief Occurs when getting an incoming call from peer.
   *
   * Local is a calle, peer is a caller.
   *
   * @param[in] node_id:     The node id(name).
   * @param[in] attach_msg:  The attach message with incoming call.
   */
  void (*on_call_request)(const char *node_id, const char *attach_msg);

  /**
   * @brief Occurs when sdk is ready to send video/audio frame.
   * This callback notifies the sender to send video/audio frame.
   */
  void (*on_start_push_frame)(void);

  /**
   * @brief Occurs when sdk stops sending video/audio frame.
   * This callback notifies the sender to stop sending video/audio frame.
   */
  void (*on_stop_push_frame)(void);

  /**
   * @brief Occurs when receiving audio frame from peer.
   *
   * @param[in] frame:        audio frame, refer to #agora_audio_frame_t.
   */
  void (*on_receive_audio_frame)(agora_audio_frame_t *frame);

  /**
   * @brief Occurs when receiving video frame from peer.
   *
   * @param[in] frame:        video frame, refer to #agora_video_frame_t.
   */
  void (*on_receive_video_frame)(agora_video_frame_t *frame);

  /**
   * @brief Occurs when peer requests a keyframe.
   *
   * This callback is optional, it is extensional.
   * This callback notifies the sender to generate a new keyframe.
   */
  void (*on_key_frame_requested)(void);

  /**
   * Occurs when network bandwidth change is detected.
   * User is expected to adjust encoder bitrate to |target_bps|
   *
   * This callback is optional, it is extensional.
   *
   * @param[in] target_bps:   Target value (bps) by which the bitrate should update
   */
  void (*on_target_bitrate_changed)(uint32_t target_bps);

  /**
   * @brief notify the status of connection
   *
   * @param status the status of connection, refer to #agora_iot_connection_status_e.
   */
  void (*on_connect_status)(uint32_t status);
} agora_iot_event_handler_t;

/* Audio codec related */
/**
 * Audio codec type list.
 */
typedef enum {
  /**
   * 0: Disable audio codec
   */
  AGORA_AUDIO_CODEC_DISABLED = 0,
  /**
   * 1: OPUS
   * - sample rate: 16000
   * - bit width: 16bit(2Byte)
   * - channel number: mono(1)
   */
  AGORA_AUDIO_CODEC_TYPE_OPUS = 1,
  /**
   * 2: G722
   * - sample rate: 16000
   * - bit width: 16bit(2Byte)
   * - channel number: mono(1)
   */
  AGORA_AUDIO_CODEC_TYPE_G722 = 2,
  /**
   * 3: G711A
   * - sample rate: 8000
   * - bit width: 16bit(2Byte)
   * - channel number: mono(1)
   */
  AGORA_AUDIO_CODEC_TYPE_G711A = 3,
  /**
   * 4: G711U
   * - sample rate: 8000
   * - bit width: 16bit(2Byte)
   * - channel number: mono(1)
   */
  AGORA_AUDIO_CODEC_TYPE_G711U = 4,
} agora_audio_codec_type_e;

/**
 * The definition of log level enum
 */
typedef enum {
  AGORA_LOG_DEFAULT = 0, // the same as AG_LOG_NOTICE
  AGORA_LOG_EMERG, // system is unusable
  AGORA_LOG_ALERT, // action must be taken immediately
  AGORA_LOG_CRIT, // critical conditions
  AGORA_LOG_ERROR, // error conditions
  AGORA_LOG_WARNING, // warning conditions
  AGORA_LOG_NOTICE, // normal but significant condition, default level
  AGORA_LOG_INFO, // informational
  AGORA_LOG_DEBUG, // debug-level messages
} agora_iot_log_level_e;

typedef struct agora_iot_log_config {
  bool disable_log;
  /**
   * 
   */
  char *log_dir;
  /**
   * refer to #agora_iot_log_level_e
   */
  uint8_t log_level;
}
agora_iot_log_config_t;

typedef struct agora_iot_config {
  char *app_id;
  char *activate_res; // the response of #agora_iot_activate
  agora_iot_event_handler_t event_handler;

  /* Extensional config */
  uint32_t audio_codec; // refer to #agora_audio_codec_type_e
  uint32_t max_possible_bitrate; // max target bitrate in on_target_bitrate_changed
  uint32_t min_possible_bitrate; // min target bitrate in on_target_bitrate_changed
  uint32_t area_code; // fill with agora_areas_type_e eg. AGO_AREA_CODE_CN | AGO_AREA_CODE_JP
  agora_iot_log_config_t log_cfg;
  bool enable_multi_users; // enable multi users
} agora_iot_config_t;

/**
 * @brief Get Version of Agora IoT SDK
 * 
 * @return String of Version
 */
const char *agora_iot_get_version(void);

/**
 * @brief Initialize a Agora IoT object, then can use Agora IoT Server to call by the Agora UID.
 * You should call agora_iot_register() to register yourself account firstly,
 * and then init SDK with your account.
 *
 * @param[in] cfg:            Refer to #agora_iot_config_t
 * @return
 * - NOT NULL: success
 * - NULL: failure
 */
int agora_iot_init(const agora_iot_config_t *cfg);

/**
 * @brief Destroy the Agora IoT object and free some resources.
 */
void agora_iot_fini(void);

/**
 * @brief Send a call request
 *
 * @param[in] peer            The peer name, which could be the binding user ID, or another device's client ID.
 * @param[in] extra_msg       The string of the extra message, the max length is 1024 bytes(include '\0').
 * @return = 0: success;
 *         < 0: failure
 */
int agora_iot_call(const char *peer, const char *extra_msg);

/**
 * @brief Answer a call request
 *
 * @return = 0: success;
 *         < 0: failure
 */
int agora_iot_answer(void);

/**
 * @brief Hangup a call request
 *
 * @return = 0: success;
 *         < 0: failure
 */
int agora_iot_hang_up(void);

/**
 * @brief Send video frame to peer.
 *
 * @param[in] frame :         Video frame to send
 * @return
 * - = 0: success
 * - < 0: failure
 */
int agora_iot_push_video_frame(agora_video_frame_t *frame);

/**
 * @brief Send audio frame to peer.
 *        Note: everytime only can write the audio data for 20ms
 *
 * @param[in] frame :         Audio frame to send
 * @return
 * - = 0: success
 * - < 0: failure
 */
int agora_iot_push_audio_frame(agora_audio_frame_t *frame);

/**
 * @brief Set RTC log file configuration, will not work if disable_rtc_log is true.
 *        Need to set if the default configuration cannot meet the requirements.
 *
 * @param[in] handle:         The reference when initialized
 * @param[in] size_per_file:  The size (bytes) of each log file.
 *                            The value range is [0, 10*1024*1024(10MB)], default 1*1024*1024(1MB).
 *                            0 means set log off
 * @param[in] max_file_count: The maximum number of log file numbers.
 *                            The value range is [0, 100], default 10.
 *                            0 means set log off
 * @return
 * - = 0: success
 * - < 0: failure
 */
int agora_iot_logfile_config(int size_per_file, int max_file_count);

#ifdef __cplusplus
}
#endif
#endif // __AGORA_IOT_API_H__