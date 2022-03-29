#pragma once

#define CONFIG_AGORA_APP_ID "d0177a34373b482a9c4eb4dedcfa586a" // Please replace with your own APP ID
#define CONFIG_USER_ACCOUNT "jack" // Please replace with your own user account
#define CONFIG_DEVICE_ID "mydoorbell" // Please replace with your own device ID
//#define CONFIG_REGISTER_NEW_DEVICE

#define CONFIG_ENABLE_LOW_POWER_MODE

#define FRAMESIZE (1) // 0 for QVGA and 1 for HVGA, 2 for VGA

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

#define CONFIG_ENABLE_RUN_TIME_STATS