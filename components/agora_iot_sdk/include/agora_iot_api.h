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

#include "agora_iot_base.h"
#include "agora_iot_call.h"
#include "agora_iot_dp.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AGO_AV_PUSH_TYPE_MASK_RTC  0x01  // Push AV data to the RTC channel
#define AGO_AV_PUSH_TYPE_MASK_OSS  0x02  // Push AV data to the OSS (Object Storage Service)

/**
 * @brief The codec types which the SDK supports, include video and audio.
 * Send the frames with the below types.
 *
 */
typedef enum {
  /**
   * 0: YUV420
   */
  AGO_VIDEO_DATA_TYPE_YUV420 = 0,

  /**
   * 1: H264
   */
  AGO_VIDEO_DATA_TYPE_H264 = 1,

  /**
   * 2: JPEG
   */
  AGO_VIDEO_DATA_TYPE_JPEG = 2,

  /**
   * 3: H265
   */
  AGO_VIDEO_DATA_TYPE_H265 = 3,

  /**
   * 10: PCM
   * support SampleRate: 16000HZ; Bits: 16; Channel: 1
   */
  AGO_AUDIO_DATA_TYPE_PCM = 10,

  /**
   * 11: OPUS
   * support SampleRate: 16000HZ; Bits: 16; Channel: 1
   */
  AGO_AUDIO_DATA_TYPE_OPUS = 11,

  /**
   * 12: PCMA
   * support SampleRate: 8000HZ; Bits: 16; Channel: 1
   */
  AGO_AUDIO_DATA_TYPE_G711A = 12,

  /**
   * 13: PCMU
   * support SampleRate: 8000HZ; Bits: 16; Channel: 1
   */
  AGO_AUDIO_DATA_TYPE_G711U = 13,

  /**
   * 14: G722
   * support SampleRate: 16000HZ; Bits: 16; Channel: 1
   */
  AGO_AUDIO_DATA_TYPE_G722 = 14,

  /**
   * 15: AACLC
   */
  AGO_AUDIO_DATA_TYPE_AACLC = 15,

  /**
   * 16: HEAAC
   */
  AGO_AUDIO_DATA_TYPE_HEAAC = 16,
} ago_av_data_type_e;

typedef struct {
  ago_av_data_type_e data_type;
  bool is_key_frame;
  uint8_t *video_buffer;
  uint32_t video_buffer_size;
} ago_video_frame_t;

typedef struct {
  ago_av_data_type_e data_type;
  uint8_t *audio_buffer;
  uint32_t audio_buffer_size;
} ago_audio_frame_t;

/**
 * @brief Agora IOT SDK Event Callback
 * relate to the video and audio.
 */
typedef struct agora_iot_rtc_callback {
  /**
   * @brief Occurs when sdk is ready to send video/audio frame.
   * This callback notifies the sender to send video/audio frame.
   *
   * @param push_type Refer to AGO_AV_PUSH_TYPE_MASK
   */
  void (*cb_start_push_frame)(uint8_t push_type);

  /**
   * @brief Occurs when sdk stops sending video/audio frame.
   * This callback notifies the sender to stop sending video/audio frame.
   *
   * @param push_type Refer to AGO_AV_PUSH_TYPE_MASK
   */
  void (*cb_stop_push_frame)(uint8_t push_type);

  /**
   * @brief Occurs when receiving audio frame from a peer user.
   *
   * @param[in] frame:        audio frame, refer to ago_audio_frame_t.
   */
  void (*cb_receive_audio_frame)(ago_audio_frame_t *frame);

  /**
   * @brief Occurs when receiving video frame from a peer user.
   *
   * @param[in] frame:        video frame, refer to ago_video_frame_t.
   */
  void (*cb_receive_video_frame)(ago_video_frame_t *frame);

  /**
   * @brief Occurs when peer requests a keyframe.
   *
   * This callback is optional, it is extensional.
   * This callback notifies the sender to generate a new keyframe.
   */
  void (*cb_key_frame_requested)(void);

  /**
   * Occurs when network bandwidth change is detected.
   * User is expected to adjust encoder bitrate to |target_bps|
   *
   * This callback is optional, it is extensional.
   *
   * @param[in] target_bps:   Target value (bps) by which the bitrate should update
   */
  void (*cb_target_bitrate_changed)(uint32_t target_bps);
} agora_iot_rtc_callback_t;

/**
 * Audio codec type list.
 */
typedef enum {
  /**
   * 0: Disable audio codec
   */
  AGO_AUDIO_CODEC_DISABLED = 0,
  /**
   * 1: OPUS
   */
  AGO_AUDIO_CODEC_TYPE_OPUS = 1,
  /**
   * 2: G722
   */
  AGO_AUDIO_CODEC_TYPE_G722 = 2,
  /**
   * 3: G711A
   */
  AGO_AUDIO_CODEC_TYPE_G711A = 3,
  /**
   * 4: G711U
   */
  AGO_AUDIO_CODEC_TYPE_G711U = 4,
} ago_audio_codec_type_e;

/**
 * Go fitting video server areas.
 */
typedef enum {
  /* the same as AREA_CODE_GLOB */
  AGO_AREA_CODE_DEFAULT = 0x00000000,
  /* Mainland China. */
  AGO_AREA_CODE_CN = 0x00000001,
  /* North America. */
  AGO_AREA_CODE_NA = 0x00000002,
  /* Europe. */
  AGO_AREA_CODE_EU = 0x00000004,
  /* Asia, excluding Mainland China. */
  AGO_AREA_CODE_AS = 0x00000008,
  /* Japan. */
  AGO_AREA_CODE_JP = 0x00000010,
  /* India. */
  AGO_AREA_CODE_IN = 0x00000020,
  /* Oceania */
  AGO_AREA_CODE_OC = 0x00000040,
  /* South-American */
  AGO_AREA_CODE_SA = 0x00000080,
  /* Africa */
  AGO_AREA_CODE_AF = 0x00000100,
  /* South Korea */
  AGO_AREA_CODE_KR = 0x00000200,
  /* The global area (except China) */
  AGO_AREA_CODE_OVS = 0xFFFFFFFE,
  /* (Default) Global. */
  AGO_AREA_CODE_GLOB = (0xFFFFFFFF),
} ago_areas_type_e;

typedef struct _agora_iot_audio_config {
  /**
   * Configure sdk built-in audio codec
   */
  ago_audio_codec_type_e audio_codec_type;
  /**
   * Pcm sample rate. Ignored if audio coded is diabled
   */
  int pcm_sample_rate;
  /**
   * Pcm channel number. Ignored if audio coded is diabled
   */
  int pcm_channel_num;
} agora_iot_audio_config_t;

typedef struct agora_iot_device_fw_info {
  /* The version information of the WiFi firmware */
  char fw_wifi_ver[16];
  /* The version information of the MCU firmware */
  char fw_mcu_ver[16];
} agora_iot_device_fw_info_t;

typedef enum {
  AGO_FW_WIFI = 0,
  AGO_FW_MCU  = 1
} agora_fw_type_e;

typedef struct agora_iot_device_fota_info {
  /* the type of the firmware, refer to agora_fw_type_e */
  agora_fw_type_e type;
  /* the size of the firmware */
  uint32_t file_size;
  /* the version of the firmware */
  char *file_ver;
  /* the URL of the firmware, can be downloaded by the URL */
  char *file_url;
} agora_iot_device_fota_info_t;

typedef struct agora_iot_ota_callback {
  /**
   * @brief notify the firmware to update by OTA
   *
   * @param info the information of the firmware.
   */
  void (*fw_updated)(const agora_iot_device_fota_info_t *info);
} agora_iot_ota_callback_t;

/**
 * Error code for RTM.
 */
typedef enum {
  /** no error */
  ERR_AGORA_RTM_OK = AGORA_ERR_SUCCESS,
  /** general error */
  ERR_AGORA_RTM_FAILED = AGORA_ERR_FAILED,
} agora_rtm_err_e;

typedef struct agora_iot_rtm_callback {
  /**
   * @brief Occurs when data comes from RTM
   *
   * @param[in] peer_uid   The remote rtm uid which the data come from.
   * @param[in] msg        The Data received.
   * @param[in] msg_len    Length of the data received.
   */
  void (*on_receive_rtm)(const char *peer_uid, const void *msg, size_t msg_len);

  /**
   * @brief Report the result of the "agora_iot_send_rtm" method call
   *
   * @param[in] msg_id     Identify one message
   * @param[in] error_code Error code number
   *                       - 0 : success
   *                       < 0 : failure
   */
  void (*on_send_rtm_result)(uint32_t msg_id, agora_rtm_err_e error_code);
} agora_iot_rtm_callback_t;

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
} agora_iot_status_e;

typedef struct agora_iot_connect_callback {
  /**
   * @brief notify the status of connection
   *
   * @param status the status of connection.
   */
  void (*on_connect_status)(agora_iot_status_e status);
} agora_iot_connect_callback_t;

typedef enum {
  /* Voice detection */
  AG_ALARM_TYPE_VAD           = 0,
  /* Motion detection */
  AG_ALARM_TYPE_MOD           = 1,
  /* Others */
  AG_ALARM_TYPE_OTHERS        = 99
} agora_iot_alarm_type_e;

typedef struct agora_iot_file_info {
  /* The suffix of the file's name, only support the format picture file, such JPG, JPEG, PNG */
  char *name_suffix;

  /* The buffer of the file */
  char *buf;

  /* The size of the file, the max size is 512kb */
  int size;
} agora_iot_file_info_t;

/**
 * The definition of log level enum
 */
typedef enum {
  AGORA_LOG_DEFAULT = 0,   // the same as AG_LOG_NOTICE
  AGORA_LOG_EMERG,         // system is unusable
  AGORA_LOG_ALERT,         // action must be taken immediately
  AGORA_LOG_CRIT,          // critical conditions
  AGORA_LOG_ERROR,         // error conditions
  AGORA_LOG_WARNING,       // warning conditions
  AGORA_LOG_NOTICE,        // normal but significant condition, default level
  AGORA_LOG_INFO,          // informational
  AGORA_LOG_DEBUG,         // debug-level messages
} agora_iot_log_level_e;

typedef struct agora_iot_config {
  /* the product and device's informations */
  char *app_id;
  char *product_key;

  char *client_id; // The unique ID for the device, after registering and will get it
  char *domain; // domain host for dp
  char *root_ca; // aws root ca buffer for dp
  char *client_crt; // client certificate buffer for dp
  char *client_key; //  client private key buffer for dp

  char *user_id; // The ID of the binding user account

  /* RTC Video and Audio */
  bool enable_rtc;
  char *certificate; // if RTC SDK enable License
  bool enable_recv_audio;
  bool enable_recv_video;
  agora_iot_rtc_callback_t rtc_cb;
  /* Extensional RTC config */
  bool disable_rtc_log; // disable low level rtc log
  char *p_log_dir;      // log save path, if it's enabled
  agora_iot_log_level_e log_level;  // log level, if it's enabled
  uint32_t max_possible_bitrate;
  bool enable_audio_config;
  agora_iot_audio_config_t audio_config;
  uint32_t area_code;   // fill with ago_areas_type_e like as AGO_AREA_CODE_CN|AGO_AREA_CODE_JP

  /* IoT Call Server */
  char *slave_server_url;
  agora_call_mode_e call_mode;
  agora_iot_call_callback_t call_cb;

  /* IoT OTA Service */
  agora_iot_ota_callback_t ota_cb;

  /* IoT RTM Service */
  agora_iot_rtm_callback_t rtm_cb;

  /* IoT server connect status */
  agora_iot_connect_callback_t connect_cb;
} agora_iot_config_t;

/**
 * @brief Initialize a Agora IoT object, then can use Agora IoT Server to call by the Agora UID.
 * You should call agora_iot_register() to register yourself account firstly,
 * and then init SDK with your account.
 *
 * @param[in] cfg:            Refer to agora_iot_config_t
 * @return
 * - NOT NULL: success
 * - NULL: failure
 */
agora_iot_handle_t agora_iot_init(const agora_iot_config_t *cfg);

/**
 * @brief Destroy the Agora IoT object and free some resources.
 *
 * @param[in] handle:         The reference when initialized
 */
void agora_iot_deinit(agora_iot_handle_t handle);

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
int agora_iot_logfile_config(agora_iot_handle_t handle, int size_per_file, int max_file_count);

/**
 * @brief Send video frame to peer.
 *
 * @param[in] handle:         The reference when initialized
 * @param[in] frame :         Video frame to send
 * @param[in] push_type:      refer to AGO_AV_PUSH_TYPE_MASK_*
 * @return
 * - = 0: success
 * - < 0: failure
 */
int agora_iot_push_video_frame(agora_iot_handle_t handle, ago_video_frame_t *frame, uint8_t push_type);

/**
 * @brief Send audio frame to peer.
 *        Note: everytime only can write the audio data for 20ms
 *
 * @param[in] handle :        The reference when initialized
 * @param[in] frame :         Audio frame to send
 * @param[in] push_type:      refer to AGO_AV_PUSH_TYPE_MASK_*
 * @return
 * - = 0: success
 * - < 0: failure
 */
int agora_iot_push_audio_frame(agora_iot_handle_t handle, ago_audio_frame_t *frame, uint8_t push_type);

/**
 * @brief Report the local version of the firmwares, such as WiFi and MCU.
 *        Note: after initializing the SDK, shoule be call this to update.
 *
 * @param[in] handle :        The reference when initialized
 * @param[in] fw_info :       The firmwares' version
 * @return
 * - = 0: success
 * - < 0: failure
 */
int agora_iot_fw_info_update(agora_iot_handle_t handle, const agora_iot_device_fw_info_t *fw_info);

/**
 * @brief Send data through Real-time Messaging (RTM) mechanism, which is a stable and reliable data channel
 *        Note: It will return failure if RTM was not login, and need not to care how to login and logout,
 *              it's controlled by APP. The sending speed allowed is limited to 60 messages per second (60qps)
 *
 * @param[in] handle   The reference when initialized
 * @param[in] peer_uid Peer RTM UID, come from "on_receive_rtm" usually
 * @param[in] msg_id   Identify the message sent
 * @param[in] msg      Message to send
 * @param[in] msg_len  Length of the message(max size: 1024 bytes)
 *
 * @return
 * - = 0: Success
 * - < 0: Failure
 */
int agora_iot_send_rtm(agora_iot_handle_t handle, const char *peer_uid, uint32_t msg_id, const void *msg, size_t msg_len);

/**
 * @brief Push the file to the OSS, now the SDK only supports the format with JPEG, JPG and PNG.
 *
 * @param[in] handle          The handle of the SDK, need to initialize firstly.
 * @param[in] file            The information of the file, refer to agora_iot_file_info_t.
 * @param[out] image_id       The image ID, while pushing to the OSS and generate a unique ID,
 *                            which will be used in the alarm event.
 * @return
 * - = 0: Success
 * - < 0: Failure
 */
int agora_iot_push_alarm_image(agora_iot_handle_t handle, const agora_iot_file_info_t *file, char **image_id);

/**
 * @brief Push the picture file while occuring a alarm event, and the picture would be related to the alarm event.
 *
 * @param[in] handle          The handle of the SDK, need to initialize firstly.
 * @param[in] begin_time      The timestamp which occured the alarm event, unit: ms.
 * @param[in] nick_name       The nick name of the device.
 * @param[in] type            The type of the alarm, refer to agora_iot_alarm_type_e.
 * @param[in] description     The description information of the alarm event.
 * @param[in] image_id        The image ID which got from the agora_iot_push_alarm_image().
 * @return
 * - = 0: Success
 * - < 0: Failure
 */
int agora_iot_push_alarm_message(agora_iot_handle_t handle, unsigned long long begin_time, const char *nick_name,
                                agora_iot_alarm_type_e type, const char *description, const char *image_id);

/**
 * @brief Start to push the video and audio frame to OSS.
 *        If successfully, and receive the notify cb_start_push_frame().
 *
 * @param[in] handle          The handle of the SDK, need to initialize firstly.
 * @param[in] begin_time      The timestamp which start to record, unit: ms.
 * @param[in] audio_type      The codec type of audio data which must matched the audio frame of agora_iot_push_audio_frame(),
 *                            otherwise pushing audio frame will failure.
 * @param[in] video_type      The codec type of video data which must matched the video frame of agora_iot_push_audio_frame(),
 *                            otherwise pushing video frame will failure.
 * @return
 * - = 0: Success
 * - < 0: Failure
 */
int agora_iot_cloud_record_start(agora_iot_handle_t handle, unsigned long long begin_time,
                                ago_av_data_type_e audio_type, ago_av_data_type_e video_type);

/**
 * @brief Stop to push the video and audio frame to OSS.
 *        If successfully, and receive the notify cb_stop_push_frame().
 *
 * @param[in] handle          The handle of the SDK, need to initialize firstly.
 * @param[in] end_time        The exact timestamp which stop to record, unit: ms.
 * @return
 * - = 0: Success
 * - < 0: Failure
 */
int agora_iot_cloud_record_stop(agora_iot_handle_t handle, unsigned long long end_time);

#ifdef __cplusplus
}
#endif
#endif // __AGORA_IOT_API_H__