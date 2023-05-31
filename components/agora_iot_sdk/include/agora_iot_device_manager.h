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

#ifndef __AGORA_IOT_DEVICE_MANAGER_H__
#define __AGORA_IOT_DEVICE_MANAGER_H__

#ifdef __cplusplus
extern "C" {
#endif

#define AGORA_IOT_CERTIFICATE_MAX_LEN 2048
#define AGORA_IOT_PRIVATE_KEY_MAX_LEN 2048
#define AGORA_IOT_DOMAIN_MAX_LEN 256
#define AGORA_IOT_CLIENT_ID_MAX_LEN 64

typedef struct agora_iot_device_info {
  /* The buffer to store device certificate from server */
  char certificate[AGORA_IOT_CERTIFICATE_MAX_LEN];
  /* The buffer to store device private key from server */
  char private_key[AGORA_IOT_PRIVATE_KEY_MAX_LEN];
  /* The buffer to store domain from server */
  char domain[AGORA_IOT_DOMAIN_MAX_LEN];
  /* The client ID for the device which will be used for the agora CALL server */
  char client_id[AGORA_IOT_CLIENT_ID_MAX_LEN];
} agora_iot_device_info_t;

/**
 * @brief Register a device and bind with target user.
 *
 * @param[in] host_url:       AWS open api host url
 * @param[in] product_key:    Product id is created when creating a product on console
 * @param[in] device_id:      The device unique ID,
 *                            Length should be no mora than 16 bytes
 *                            Supported character scopes are:
 *                            - The 26 English letters: A to Z or a to z
 *                            - The 10 numbers: 0 to 9
 * @param[in] user_id:        The user id from application.
 *                            If you only need the pure call-server, the user_id should be NULL.
 * @param[in] device_nickname:The device nickname on mobile phone to display.
 *                            If you only need the pure call-server, the user_id should be NULL.
 * @param[out] info:          The returned information for sdk init
 * @return
 * - = 0: success;
 * - < 0: failure, refer to agora_iot_error_e
 */
int agora_iot_register_and_bind(const char *host_url, const char *product_key, const char *device_id,
                                const char *user_id, const char *device_nickname, agora_iot_device_info_t *info);

/**
 * @brief Query the user binding with device
 * If you only need the call-server, you should not call this API due to it would return failure in the pure call-server.
 *
 * @param[in] host_url:       AWS open api host url
 * @param[in] product_key:    Product id is created when creating a product on console
 * @param[in] device_id:      The device unique ID
 *                            Length should be no mora than 16 bytes
 *                            Supported character scopes are:
 *                            - The 26 English letters: A to Z or a to z
 *                            - The 10 numbers: 0 to 9
 * @param[out] user_id:       The user id, length: 64 bytes
 * @param[in] id_len          The length of the user id buffer, include the terminal '\0'.
 * @return
 * - = 0: success
 * - < 0: failure, refer to agora_iot_error_e
 */
int agora_iot_query_user(const char *host_url,  const char *product_key, const char *device_id, char *user_id, int id_len);

/**
 * @brief Activate agora RTC license, this function cannot reentrant
 *
 * @param[in] appid       appid used for this product, got from Agora Developer Platform
 * @param[in] key         customer key of bisic auth, got from Agora Developer Platform
 * @param[in] secret      customer secret of bisic auth, got from Agora Developer Platform
 * @param[in] product_key Product id is created when creating a product on console
 * @param[in] device_id   The device unique ID, eg. MAC address,
 *                        Length should be less than 64 bytes
 *                        Supported character scopes are:
 *                        - The 26 uppercase English letters: A to Z
 *                        - The 10 numbers: 0 to 9
 * @param[in] pid         The PID of agora license, it will be ignore on v1.0 version
 *                        got from agora license managment platform: 
 *                        - https://console.agora.io/license/usage
 * @param[out] cert     the buffer of keeping the license certificate,
 *                      should be save to file or flash, and YOU HAVE TO FREE IT AFTER.
 * @return = 0: success;
 *         < 0: failure, refer to agora_iot_error_e
 */
int agora_iot_license_activate(const char *appid, const char *key, const char *secret,
                              const char *product_key, const char *device_id, const char *pid, char **cert);

#ifdef __cplusplus
}
#endif

#endif /* __AGORA_IOT_DEVICE_MANAGER_H__ */