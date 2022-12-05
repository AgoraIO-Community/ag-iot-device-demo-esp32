#ifndef __DOORBELL_DP_COMMON_H__
#define __DOORBELL_DP_COMMON_H__

#include "agora_iot_api.h"
#include "agora_iot_base.h"
#include "device_state.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_USE_OPUS_CODEC)
#define AUDIO_CODEC_TYPE AGO_AUDIO_CODEC_TYPE_OPUS
#define CONFIG_PCM_SAMPLE_RATE (16000)
#define CONFIG_PCM_DATA_LEN  640
#elif defined(CONFIG_USE_G722_CODEC)
#define AUDIO_CODEC_TYPE AGO_AUDIO_CODEC_TYPE_G722
#define CONFIG_PCM_SAMPLE_RATE (16000)
#define CONFIG_PCM_DATA_LEN  640
#elif defined(CONFIG_USE_G711U_CODEC)
#define AUDIO_CODEC_TYPE AGO_AUDIO_CODEC_TYPE_G711U
#define CONFIG_PCM_SAMPLE_RATE (8000)
#define CONFIG_PCM_DATA_LEN  320
#elif defined(CONFIG_USE_OPUSFB_CODEC)
#define AUDIO_CODEC_TYPE AGO_AUDIO_CODEC_TYPE_OPUS
#define CONFIG_PCM_SAMPLE_RATE (48000)
#define CONFIG_PCM_DATA_LEN  1920
#endif

#define SEND_AUDIO_DATA_TYPE (10) // 10 for PCM, 13 for G711U
#define SEND_VIDEO_DATA_TYPE (2)  // 1 for H264 and 2 for JPEG, 3 for H265


typedef enum {
  SYS_UP_MODE_NONE = 0,
  SYS_UP_MODE_RESTORE,
  SYS_UP_MODE_POWERON,
  SYS_UP_MODE_WAKEUP,
  SYS_UP_MODE_MAX
} sys_up_mode_e;

char *get_client_id(const char *product_key, const char *device_id);

char *get_device_id(void);

device_handle_t load_devic_state(void);

int save_device_state(device_handle_t dev_state);

int activate_device(device_handle_t dev_state);

int update_device_work_state(agora_iot_handle_t handle, sys_up_mode_e mode);

void update_device_low_power(agora_iot_handle_t handle);

int start_alarm_record(agora_iot_handle_t handle, unsigned long long begin_time);

int stop_alarm_record(agora_iot_handle_t handle);

int alarm_message_send(agora_iot_handle_t handle, unsigned long long begin_time, agora_iot_file_info_t file_info, char *nick_name, agora_iot_alarm_type_e alarm_type, char *alarm_desc);

#ifdef __cplusplus
}
#endif

#endif // __DOORBELL_DP_COMMON_H__