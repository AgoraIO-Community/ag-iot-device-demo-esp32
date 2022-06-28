#ifndef __DEVICE_STATE_H__
#define __DEVICE_STATE_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef void * device_handle_t;

/**
 * @brief Create a device state config, if there is not any state config in device system
 *
 * @param None
 * @return NULL or 0 means faild
 */
device_handle_t device_create_state(void);

/**
 * @brief Load device state config content, it's JSON type and got from a file or flash
 *
 * @param content The content of device state config, should be JSON type
 * @return NULL or 0 means faild
 */
device_handle_t device_load_state(const char *content);

 /**
 * @brief Build content string to save, it's a string with JSON type
 *
 * @param handle device state handle from load interface
 * @return Content string with all state items, save it to file or flash, and must free it youself
 */
const char* device_build_state_content(device_handle_t handle);

 /**
 * @brief Destroy device state config, will free all resource that used for process
 *
 * @param handle device state handle from load interface
 * @return Nothing
 */
void device_destroy_state(device_handle_t handle);

 /**
 * @brief Get state items from config, select a interface according to value type
 *
 * @param handle device state handle from load interface
 * @param key the key of item
 * @param value a pointer used to save value of item, must free it youself for string
 * @return 0: success;
 *          < 0: failure
 */
int device_get_item_int(device_handle_t handle, const char *key, int *value);
int device_get_item_double(device_handle_t handle, const char *key, double *value);
int device_get_item_string(device_handle_t handle, const char *key, char **value);

 /**
 * @brief Set state items from config, select a interface according to value type
 *
 * @param handle device state handle from load interface
 * @param key the key of item
 * @param value  value of item
 * @return 0: success;
 *          < 0: failure
 */
int device_set_item_int(device_handle_t handle, const char *key, int value);
int device_set_item_double(device_handle_t handle, const char *key, double value);
int device_set_item_string(device_handle_t handle, const char *key, const char *value);

#ifdef __cplusplus
}
#endif

#endif // __DEVICE_STATE_H__