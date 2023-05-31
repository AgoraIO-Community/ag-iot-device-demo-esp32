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

#ifndef __AGORA_IOT_DP_H__
#define __AGORA_IOT_DP_H__

#include "agora_iot_base.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  /* No Error */
  ERR_AG_DP_SUCCESS           = AGORA_ERR_SUCCESS,
  /* General error (no specified reason). */
  ERR_AG_DP_FAILED            = AGORA_ERR_FAILED,
  /* Invalid parameter called */
  ERR_AG_DP_INVALID_ARGUMENT  = AGORA_ERR_INVALID_ARGUMENT,
} agora_iot_dp_result_e;

#ifndef __AGORA_IOT_DP_DEFS_H__
#define __AGORA_IOT_DP_DEFS_H__

#define AGORA_DP_USER_LEN     64

/* Data Point Type Start */
typedef enum agora_dp_type {
  AGORA_DP_TYPE_INT           = 0,
  AGORA_DP_TYPE_BOOL          = 1,
  AGORA_DP_TYPE_ENUM          = 2,
  AGORA_DP_TYPE_STR           = 3,
} agora_dp_type_e;
/* Data Point Type End */

#define AGORA_DP_ID_MIN       100

typedef union {
  int dp_int;
  bool dp_bool;
  unsigned int dp_enum;
  /*
   * Attention: You have to free it youself, 
                It is recommended to use global variables for on_dp_query_callback(),
   *            maybe memory leak if you malloc a new block, 
   *            and never use local variables that will be destroyed when callback function return
   */
  char *dp_str;
} agora_dp_value_t;

typedef struct agora_dp_info {
  /* Data point ID must >= AGORA_DP_ID_MIN */
  unsigned int dp_id;
  /* Refer to AGORA_DP_TYPE_XXX */
  agora_dp_type_e dp_type;
  agora_dp_value_t dp_value;
} agora_dp_info_t;

/* Occurs when query the local state of data point.
 * You must get local value of device, and set it to agora_dp_info
 *
 * @param[in/out] info:       Set local value to this data point correctly
 * @param[in] args:           The args when register
 */
typedef void (*on_dp_query_callback)(agora_dp_info_t *info, void *args);

/* Occurs when command from remote.
 * You can set the value from remote to local device or what else you want to do.
 *
 * @param[in] info:           Data point value from remote
 * @param[in] args:           The args when register
 */
typedef void (*on_dp_cmd_callback)(const agora_dp_info_t *info, void *args);
#endif

/**
 * Register a query callback for target data point
 *
 * @param[in] handle:         The reference when initialized
 * @param[in] dp_id:          The data point ID, reference to agora_iot_dp_defs.h.
 * @param[in] dp_type:        The data point type.
 * @param[in] callback:       Occurs when app or cloud query the data point of dp_id
 * @param[in] args:           Args pass to callback
 * @return
 * - = 0: success
 * - < 0: failure, refer to agora_iot_dp_result_e
 */
agora_iot_dp_result_e agora_iot_dp_register_dp_query_handler(agora_iot_handle_t handle, int dp_id,
                                                             agora_dp_type_e dp_type, on_dp_query_callback callback,
                                                             void *args);

/**
 * Register a command callback for target data point
 *
 * @param[in] handle:         The reference when initialized
 * @param[in] dp_id:          The data point ID, >= AGORA_DP_ID_MIN.
 * @param[in] dp_type:        The data point type, refer to AGORA_DP_TYPE_XXX.
 * @param[in] callback:       Occurs when app or cloud set the data point value of dp_id
 * @param[in] args:           Args pass to callback
 * @return
 * - = 0: success
 * - < 0: failure, refer to agora_iot_dp_result_e
 */
agora_iot_dp_result_e agora_iot_dp_register_dp_cmd_handler(agora_iot_handle_t handle, int dp_id,
                                                           agora_dp_type_e dp_type, on_dp_cmd_callback callback,
                                                           void *args);

/**
 * Publish one data point information
 *
 * @param[in] handle:         The reference when initialized
 * @param[in] info:           The point of data point infomation
 * @return
 * - = 0: success
 * - < 0: failure, refer to agora_iot_dp_result_e
 */
agora_iot_dp_result_e agora_iot_dp_publish(agora_iot_handle_t handle, agora_dp_info_t *info);

/**
 * @brief Publish all of the registered data points.
 * It will get the value of every data point in the query callback.
 * You should implement the query callback for every data point,
 * and don't block in the callback.
 *
 * @param handle              The reference when initialized
 * @return agora_iot_dp_result_e
 */
agora_iot_dp_result_e agora_iot_dp_publish_all(agora_iot_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* __AGORA_IOT_DP_H__ */