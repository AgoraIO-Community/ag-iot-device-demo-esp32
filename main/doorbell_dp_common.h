#ifndef __DOORBELL_DP_COMMON_H__
#define __DOORBELL_DP_COMMON_H__

#include "agora_iot_api.h"
#include "device_state.h"
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The internal audio encoder codec type
 *
 * The SDK supports encode the PCM data before sending.
 * The codec type of encoder refer to the definition of ago_audio_codec_type_e
 * 0 -> Disable encoder
 * 1 -> OPUS
 * 2 -> G722
 * 3 -> G711A
 * 4 -> G711U
 *
 * Note: The Agora OSS doesn't support OPUS yet.
 */
#define INTERNAL_AUDIO_ENC_TYPE (4)

#if (INTERNAL_AUDIO_ENC_TYPE == 0)
#define CONFIG_PCM_SAMPLE_RATE (16000)
#define CONFIG_PCM_CHANNEL_NUM (1)
#define CONFIG_PCM_DATA_LEN 640
#elif (INTERNAL_AUDIO_ENC_TYPE == 1)
#define CONFIG_PCM_SAMPLE_RATE (16000)
#define CONFIG_PCM_CHANNEL_NUM (1)
#define CONFIG_PCM_DATA_LEN  640
#elif (INTERNAL_AUDIO_ENC_TYPE == 2)
#define CONFIG_PCM_SAMPLE_RATE (16000)
#define CONFIG_PCM_CHANNEL_NUM (1)
#define CONFIG_PCM_DATA_LEN  640
#elif (INTERNAL_AUDIO_ENC_TYPE == 3)
#define CONFIG_PCM_SAMPLE_RATE (8000)
#define CONFIG_PCM_CHANNEL_NUM (1)
#define CONFIG_PCM_DATA_LEN  320
#elif (INTERNAL_AUDIO_ENC_TYPE == 4)
#define CONFIG_PCM_SAMPLE_RATE (8000)
#define CONFIG_PCM_CHANNEL_NUM (1)
#define CONFIG_PCM_DATA_LEN  320
#endif

#define SEND_AUDIO_DATA_TYPE (10) // 10 for PCM, 13 for G711U
#define SEND_VIDEO_DATA_TYPE (2)  // 1 for H264 and 2 for JPEG, 3 for H265

#define DEFAULT_MAX_BITRATE (2500000)
#define DEFAULT_MIN_BITRATE (500000)


typedef enum {
  SYS_UP_MODE_NONE = 0,
  SYS_UP_MODE_RESTORE,
  SYS_UP_MODE_POWERON,
  SYS_UP_MODE_WAKEUP,
  SYS_UP_MODE_MAX
} sys_up_mode_e;

//char *get_device_id(void);

const char *get_node_id(void);

const char *get_node_secret(void);

device_handle_t load_devic_state(void);

int save_device_state(device_handle_t dev_state);

int activate_device(device_handle_t dev_state);

#ifdef __cplusplus
}
#endif

#endif // __DOORBELL_DP_COMMON_H__