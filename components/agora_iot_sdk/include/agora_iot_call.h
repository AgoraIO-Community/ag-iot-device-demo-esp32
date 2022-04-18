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

#ifndef _AGORA_IOT_CALL_H_
#define _AGORA_IOT_CALL_H_

#include "agora_iot_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send a call request
 *
 * @param handle The instance of Agora IoT
 * @param peer The peer's account which is called,
 *              if don't know the uid of the account, keep it to be 0.
 *              When sucess, the uid will be update to a correct value.
 * @param cnt The count of the peers' account, 1 <= cnt <=60
 * @param session_id The ID of the session.
 * @param extra_msg The string of the extra message, the max length is 1024 bytes(include '\0').
 * @return 0: success;
 *          < 0: failure
 */
int agora_iot_call(agora_iot_handle_t handle, agora_iot_account *peer, int cnt, const char *extra_msg);

/**
 * @brief Answer a call request
 *
 * @param handle The instance of Agora IoT
 * @return 0: success;
 *          < 0: failure
 */
int agora_iot_answer(agora_iot_handle_t handle);

/**
 * @brief Hangup a call session
 *
 * @param handle The instance of Agora IoT
 * @return 0: success;
 *          < 0: failure
 */
int agora_iot_hang_up(agora_iot_handle_t handle);

/**
 * @brief Alarm to others and start cloud recording
 *
 * @param handle The instance of Agora IoT
 * @param peer The others which is alarmed
 *              if don't know the uid of the account, keep it to be 0.
 *              When sucess, the uid will be update to a correct value.
 * @param cnt The count of the peers' account, 1 <= cnt <=60
 * @param session_id The ID of the session.
 * @param extra_msg The string of the extra message, the max length is 1024 bytes(include '\0').
 * @param type The type of alarm.
 * @return 0: success;
 *          < 0: failure
 */
int agora_iot_alarm(agora_iot_handle_t handle, agora_iot_account *peer, int cnt, const char *extra_msg,
                    agora_iot_alarm_type_e type);

#ifdef __cplusplus
}
#endif

#endif // _AGORA_IOT_CALL_H_