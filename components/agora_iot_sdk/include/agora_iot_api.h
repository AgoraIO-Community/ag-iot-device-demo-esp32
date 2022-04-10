#ifndef __AGORA_IOT_API_H__
#define __AGORA_IOT_API_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AGORA_IOT_ACCOUNT_NAME_LENGTH 64
#define AGORA_IOT_CHANNEL_LENGTH 64
#define AGORA_IOT_TOKEN_LENGTH 1024

typedef void *agora_iot_handle_t;

// Error Code define
typedef enum {
  ERROR_OK = 0,
  ERROR_PARAM_INVALID = -1,
  ERROR_INIT_FAILED = -2,
  ERROR_STATE = -3,

  // Detail for Network
  ERROR_NETWORK = -10, // general network error
  ERROR_NETWORK_TIMEOUT = -11, // network communicate timeoout
  ERROR_NETWORK_DATA = -12, // network data has wrong format or can not analyze

  // Error IOT Server
  ERROR_SERVER_CONN = -20, // general IOT Server connection error
  ERROR_SERVER_VERIFY = -21, // IOT Server CA verify failed

  // Error for Device
  ERROR_DEVICE_UNREGISTER = -30,

  // Error for User
  ERROR_USER_UNREGISTER = -50,

  ERROR_REGISTER = -99,
  ERROR_UNDEFINE = -100
} agora_iot_error_code;

typedef enum { TYPE_DEVICE = 0, TYPE_USER = 1 } agora_iot_account_type;

typedef enum {
  ALARM_TYPE_INVALID = 0,
  ALARM_TYPE_PASS_BY = 1, // someone passes by
  ALARM_TYPE_MOVING = 2, // something moves
  ALARM_TYPE_NOSIE = 3 // alarm of nosie
} agora_iot_alarm_type_e;

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
     * 10: PCM
     */
  AGO_AUDIO_DATA_TYPE_PCM = 10,
  /**
     * 11: OPUS
     */
  AGO_AUDIO_DATA_TYPE_OPUS = 11,
  /**
    * 12: PCMA
    */
  AGO_AUDIO_DATA_TYPE_G711A = 12,
  /**
    * 13: PCMU
    */
  AGO_AUDIO_DATA_TYPE_G711U = 13,
  /**
     * 14: G722
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

typedef struct _agora_iot_account {
  agora_iot_account_type type;
  // The Agora UID, can be used in join channel with token
  uint32_t uid;
  // The name of a device or a user
  char name[AGORA_IOT_ACCOUNT_NAME_LENGTH];
} agora_iot_account;

typedef struct _agora_iot_call_session {
  uint32_t id;
  char channel[AGORA_IOT_CHANNEL_LENGTH];
  char token[AGORA_IOT_TOKEN_LENGTH];
} agora_iot_call_session;

typedef struct _agora_iot_call_msg {
  agora_iot_account *peer;
  agora_iot_call_session *session;
  char *attach_msg;
} agora_iot_call_msg;

// Callback event
typedef struct _agora_iot_callback {
  /* Notification from Agora IoT Server */
  void (*cb_login_success)();
  void (*cb_call_response)(int result, const agora_iot_call_session *session);
  void (*cb_call_request)(const char *peer_name, const char *attach_msg);
  void (*cb_start_push_frame)();
  void (*cb_stop_push_frame)();

  /* Notification from a remote user or device */
  void (*cb_call_answered)(const char *peer_name);
  void (*cb_call_hung_up)(const char *peer_name);
  void (*cb_call_busy)(const char *peer_name);
  /* When other call you, but you doesn't response and then you will receive the notify of timeout from him */
  void (*cb_call_timeout)(const char *peer_name);
  /* When you call others, but no one responses and then you will receive the notify */
  void (*cb_call_all_timeout)(void);

  /* customize message from other devices or users */
  void (*cb_call_custom_msg)(const char *peer_name, const char *msg, int len);

  void (*cb_receive_audio_frame)(ago_audio_frame_t *frame);
  void (*cb_receive_video_frame)(ago_video_frame_t *frame);

  /**
   * Occurs when remote requests a keyframe.
   * This callback notifies the sender to generate a new keyframe.
   */
  void (*cb_key_frame_requested)(void);
  /**
   * Occurs when network bandwidth change is detected. User is expected to adjust encoder
   * bitrate to |target_bps|
   *
   * @param[in] target_bps Target value (bps) by which the bitrate should update
   */
  void (*cb_target_bitrate_changed)(uint32_t target_bps);

  void (*cb_connect_state)(int state);
} agora_iot_callback_t;

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

typedef struct agora_iot_config {
  /* the product and device's informations */
  const char *app_id;
  const char *cloud_rec_basic_auth;
  const char *product_id;
  const char *device_id; // for Apical, use the device's MAC as ID
  const char *domain; // domain host for dp
  const char *root_ca; // aws root ca buffer for dp
  const char *client_crt; // client certificate buffer for dp
  const char *client_key; //  client private key buffer for dp
  uint16_t region;

  agora_iot_account *self;

  /* RTC Video and Audio */
  bool enable_rtc;
  const char *certificate; // if RTC SDK enable License
  bool enable_recv_audio;
  bool enable_recv_video;
  bool disable_rtc_log; // disable low level rtc log
  uint32_t max_possible_bitrate; // the maxium possible bitrate for the video encoder

  agora_iot_callback_t *cb;
  agora_iot_audio_config_t *audio_config;
} agora_iot_config_t;

/**
 * @brief Register a device or a user, if success will get a new Agora UID.
 *
 * @param[in] app_id The application ID of the Agora when register your application
 * @param[in,out] info The information of a iot account returned when register successfully
 * @return 0: success;
 *          < 0: failure
 * @deprecated Will be removed while use MQTT instead of RTM
 */
int agora_iot_register(const char *app_id, agora_iot_account *info);

/**
 * @brief Initialize a Agora IoT object, then can use Agora IoT Server to call by the Agora UID.
 * You should call agora_iot_register() to register yourself account firstly,
 * and then init SDK with your account.
 *
 * @param[in] cfg: Refer to agora_iot_config_t
 * @return NOT NULL: success
 *         NULL: failure
 */
agora_iot_handle_t agora_iot_init(const agora_iot_config_t *cfg);

/**
 * @brief Destory the Agora IoT object and free some resources.
 *
 * @param[in] handle: The reference when initialized
 */
void agora_iot_deinit(agora_iot_handle_t handle);

/**
 * @brief Send video frame to remote.
 *
 * @param[in] handle: The reference when initialized
 * @param[in] frame :Video frame to send
 * @return int : 0 success
 *               < 0: failure
 */
int agora_iot_push_video_frame(agora_iot_handle_t handle, ago_video_frame_t *frame);

/**
 * @brief Send audio frame to remote.
 *
 * @param[in] handle : The reference when initialized
 * @param[in] frame : Audio frame to send
 * @return int : 0 success
 *               < 0: failure
 */
int agora_iot_push_audio_frame(agora_iot_handle_t handle, ago_audio_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif // __AGORA_IOT_API_H__