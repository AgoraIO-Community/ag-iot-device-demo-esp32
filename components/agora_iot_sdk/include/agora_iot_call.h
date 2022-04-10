/*************************************************************
 * Author:  Xianbin Sun (sunxianbin@agora.io)
 * Date  :  2022/2/14
 * Module:  Agora IoT Call
 *
 *
 * This is a header file of Agora IoT Call
 * Copyright (C) 2022 Agora IO
 * All rights reserved.
 *
 *************************************************************/
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