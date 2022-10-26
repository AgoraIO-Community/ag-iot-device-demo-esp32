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

#ifndef __AGORA_IOT_BASE_H__
#define __AGORA_IOT_BASE_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef void* agora_iot_handle_t;

// Error Code define
typedef enum {
  /* No Error */
  AGORA_ERR_SUCCESS                                   = 0,

  /* General error (no specified reason). */
  AGORA_ERR_FAILED                                    = -1,

  /* Invalid argument called */
  AGORA_ERR_INVALID_ARGUMENT                          = -2,

  /* The required component is not enabled */
  AGORA_ERR_COMPONENT_NOT_ENABLED                     = -3,

  /* The feature is disabled */
  AGORA_ERR_FEATURE_DISABLED                          = -4
} agora_iot_error_e;

#ifdef __cplusplus
}
#endif

#endif // __AGORA_IOT_BASE_H__