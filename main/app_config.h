#pragma once

#define CONFIG_AGORA_APP_ID "24cbe3f57dxxxxxxxxxxxxxxxxf3ff03" // Please replace with your own APP ID

// Found product key form device manager platform
#define CONFIG_PRODUCT_KEY "EJIJExxxxxxxx5lI4"

#define CONFIG_USER_ID "68753xxxxxxxxx3440" // Please replace with your own user ID
#define CONFIG_NODE_ID "01H5KT6xxxxxxxxxCV43PNMK" // Please replace with your own node ID
#define CONFIG_NODE_SECRET "78c96bdebxxxxxxxxxxxxa5d1e2b" // Please replace with your own node secret


// Agora Master Server URL
#define CONFIG_MASTER_SERVER_URL "https://app.agoralink-iot-cn.sd-rtn.com"

// Agora Slave Server URL
#define CONFIG_SLAVE_SERVER_URL "https://api.agora.io/agoralink/cn/api"

// Device cert file size max
#define CERT_BUF_SIZE_MAX (1024 * 2)

#define CONFIG_SEND_PCM_DATA
#define CONFIG_ENABLE_RUN_TIME_STATS

#define CONFIG_BLUFI_ENABLE

#ifndef CONFIG_AUDIO_ONLY
// #define UVC_STREAM_ENABLE
#endif

#define NVS_STORAGE
// #define CONFIG_SDCARD

// #define CONFIG_LCD_DISPLAY
#ifdef CONFIG_LCD_DISPLAY
// #define CONFIG_RTC_DISPLAY    /* display remote frame */
#define FRAMESIZE (0) /* 0 for QVGA and 1 for HVGA, 2 for VGA */
#else
#define FRAMESIZE (1) /* 0 for QVGA and 1 for HVGA, 2 for VGA */
#endif


// Device Firmware version
#define CONFIG_FM_WIFI_VER "1.0.0"
#define CONFIG_FM_MCU_VER "1.0.0"

// 0 for no sleep
#define CONFIG_NO_SLEEP             0
// 1 for light sleep
#define CONFIG_ENABLE_LIGHT_SLEEP   1
// 2 for deep sleep
#define CONFIG_ENABLE_DEEP_SLEEP    2

#define LOWER_POWER_MODE   (CONFIG_NO_SLEEP)