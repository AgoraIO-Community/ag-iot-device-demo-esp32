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

#include "agora_iot_base.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  /* No Error */
  ERR_AG_CALL_SUCCESS            = AGORA_ERR_SUCCESS,
  /* General error (no specified reason). */
  ERR_AG_CALL_FAILED             = AGORA_ERR_FAILED,
  /* Invalid parameter called */
  ERR_AG_CALL_INVALID_ARGUMENT   = AGORA_ERR_INVALID_ARGUMENT,

  /* Peer already in call error */
  ERR_AG_CALL_PEER_BUSY          = -100001,
  /* No incoming call, can not answer */
  ERR_AG_CALL_CAN_NOT_ANSWER     = -100002,
  /* Not in call, outgoing call or incoming call, cant not hangup */
  ERR_AG_CALL_CAN_NOT_HANGUP     = -100003,
  /* Wait timeout */
  ERR_AG_CALL_PEER_TIMEOUT       = -100004,
  /* Already incoming call or outgoing call */
  ERR_AG_CALL_IS_CALLING         = -100005,
  /* Illegal answer operation(e.g. outgoing call & answer) */
  ERR_AG_CALL_ILLEGAL_ANSWER     = -100006,
  /* Caller ID is same with callee ID */
  ERR_AG_CALL_CAN_NOT_CALL_YOURSELF = -100007,

  /* The cloud record is starting, can not start again. */
  ERR_AG_CLOUD_REC_STARTED        = -200001,
  /* The cloud record is stopped, can not stop again. */
  ERR_AG_CLOUD_REC_STOPPED        = -200002,
  /* The cloud record is not permission. */
  ERR_AG_CLOUD_REC_NOT_PERMISSION = -200003,
  /* The permission of the cloud record is expired */
  ERR_AG_CLOUD_REC_PERMISSION_EXPIRED = -200004,
  /* The shadow of the device is without the appID */
  ERR_AG_CLOUD_REC_WITHOUT_APPID  = -200005,
  /* The Agora IoT server does not enable */
  ERR_AG_CLOUD_REC_SERVER_NOT_ENABLE  = -200006,

  /* There isn't the permission of the alarm server */
  ERR_AG_ALARM_NOT_PERMISSION     = -999997,
  /* Invalid arguments for the alarm server */
  ERR_AG_ALARM_INVALID_ARGUMENTS  = -999998,

  /* Unexpectedly server error */
  ERR_AG_SERVER_UNEXPECTEDLY_ERROR = -999999,
} agora_iot_call_result_e;

/**
 * @brief The mode of calling
 *
 */
typedef enum {
  CALL_MODE_SINGLE    = 1,  // 1 device vs 1 user
  CALL_MODE_MUTLI     = 2,  // 1 device vs mutli users
} agora_call_mode_e;

typedef struct agora_iot_call_callback {
  /******************** Event Callback related to Call *******************/
  /**
    * @brief Occurs when getting an incoming call from a peer user.
    *
    * Local is a calle, peer is a caller.
    *
    * @param[in] peer_id:     The peer id(name).
    * @param[in] attach_msg:  The attach message with incoming call.
    */
  void (*cb_call_request)(const char *peer_id, const char *attach_msg);

  /**
    * @brief Occurs when peer as a callee answer call.
    *
    * Local is a caller, peer is a callee.
    *
    * @param[in] peer_id:     The peer id.
    */
  void (*cb_call_answered)(const char *peer_id);

  /**
    * @brief Occurs when peer as a callee deny the call.
    *
    * Local is a caller, peer is a callee.
    *
    * @param[in] peer_id:     The peer id.
    */
  void (*cb_call_hung_up)(const char *peer_id);

  /**
    * @brief Occurs when incoming call, but local do not answer or hung up.
    *
    * Local is a callee, peer is a caller.
    *
    * @param[in] peer_id:     The peer id.
    */
  void (*cb_call_local_timeout)(const char *peer_id);

  /**
    * @brief Occurs when outgoing call, but peer do not answer or hung up.
    *
    * Local is a caller, peer is a callee.
    *agora_call_mode_e
    * @param[in] peer_id:     The peer id.
    */
  void (*cb_call_peer_timeout)(const char *peer_id);
  /***********************************************************************/
} agora_iot_call_callback_t;

/**
 * @brief Send a call request
 *
 * @param[in] handle          The instance of Agora IoT
 * @param[in] peer            The peer name, which could be the binding user ID, or another device's client ID.
 * @param[in] extra_msg       The string of the extra message, the max length is 1024 bytes(include '\0').
 * @return = 0: success;
 *         < 0: failure, refer to agora_iot_call_result_e
 */
agora_iot_call_result_e agora_iot_call(agora_iot_handle_t handle, const char *peer, const char *extra_msg);

/**
 * @brief Answer a call request
 *
 * @param[in] handle          The instance of Agora IoT
 * @return = 0: success;
 *         < 0: failure, refer to agora_iot_call_result_e
 */
agora_iot_call_result_e agora_iot_answer(agora_iot_handle_t handle);

/**
 * @brief Hangup a call request
 *
 * @param[in] handle          The instance of Agora IoT
 * @return = 0: success;
 *         < 0: failure, refer to agora_iot_call_result_e
 */
agora_iot_call_result_e agora_iot_hang_up(agora_iot_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* _AGORA_IOT_CALL_H_ */