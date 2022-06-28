#ifndef __DOORBELL_DP_COMMON_H__
#define __DOORBELL_DP_COMMON_H__

#include "agora_iot_base.h"
#include "device_state.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
  SYS_UP_MODE_NONE = 0,
  SYS_UP_MODE_RESTORE,
  SYS_UP_MODE_POWERON,
  SYS_UP_MODE_WAKEUP,
  SYS_UP_MODE_MAX
} sys_up_mode_e;

char *get_client_id(const char *product_key, const char *device_id);

char *get_device_id(void);

device_handle_t load_devic_state(void);

int save_device_state(device_handle_t dev_state);

int activate_device(device_handle_t dev_state);

int update_device_work_state(agora_iot_handle_t handle, sys_up_mode_e mode);

void update_device_low_power(agora_iot_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // __DOORBELL_DP_COMMON_H__