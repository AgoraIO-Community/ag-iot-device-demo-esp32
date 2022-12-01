#pragma once

#define CONFIG_AGORA_APP_ID "4b31fcXXXXXXXXXXXXX3037" // Please replace with your own APP ID
#define CONFIG_CUSTOMER_KEY "8620fdXXXXXXXXXXX7363"
#define CONFIG_CUSTOMER_SECRET "492c1XXXXXXXXXe802"
#define CONFIG_LICENSE_PID "00F8DXXXXXXXXXX2646"

// Agora Master Server URL
#define CONFIG_MASTER_SERVER_URL "https://app.agoralink-iot-cn.sd-rtn.com"

// Agora Slave Server URL
#define CONFIG_SLAVE_SERVER_URL "https://api.agora.io/agoralink/cn/api"

// Found product key form device manager platform
#define CONFIG_PRODUCT_KEY "EJIJXXXXXXXXlI4"

// Device cert file size max
#define CERT_BUF_SIZE_MAX (1024 * 2)

// AWS service root ca, it will never be changed
#define CONFIG_AWS_ROOT_CA                                                                                                    \
  "-----BEGIN CERTIFICATE-----\r\n\
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\r\n\
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\r\n\
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\r\n\
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\r\n\
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\r\n\
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\r\n\
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\r\n\
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\r\n\
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\r\n\
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\r\n\
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\r\n\
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\r\n\
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\r\n\
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\r\n\
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\r\n\
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\r\n\
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\r\n\
rqXRfboQnoZsG4q5WTP468SQvvG5\r\n\
-----END CERTIFICATE-----\r\n"

#define CONFIG_USE_G711U_CODEC
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