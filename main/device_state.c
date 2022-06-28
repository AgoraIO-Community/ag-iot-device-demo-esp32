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

#include "device_state.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "cJSON.h"

#define JSON_HANDLE(handle) (cJSON *)(handle)
#define VOID_HANDLE(root) (device_handle_t)(root)

device_handle_t device_create_state(void) {
    cJSON *root = cJSON_CreateObject();
    return VOID_HANDLE(root);
}

device_handle_t device_load_state(const char *content) {
    if (NULL == content) {
        return NULL;
    }
    cJSON *root = cJSON_Parse(content);
    return VOID_HANDLE(root);
}

const char* device_build_state_content(device_handle_t handle)
{
    cJSON *root = JSON_HANDLE(handle);
    if (NULL == root) {
        return NULL;
    }
    return cJSON_Print(root);
}

void device_destroy_state(device_handle_t handle)
{
    cJSON *root = JSON_HANDLE(handle);
    if (root) {
        cJSON_Delete(root);
    }
}

int device_get_item_int(device_handle_t handle, const char *key, int *value)
{
    cJSON *root = JSON_HANDLE(handle);
    if (NULL == root || NULL == key || NULL == value) {
        return -1;
    }

    cJSON *js_item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (cJSON_IsNumber(js_item)) {
        *value = (int)cJSON_GetNumberValue(js_item);
        return 0;
    } else {
        return -1;
    }
}

int device_get_item_double(device_handle_t handle, const char *key, double *value)
{
    cJSON *root = JSON_HANDLE(handle);
    if (NULL == root || NULL == key || NULL == value) {
        return -1;
    }

    cJSON *js_item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (cJSON_IsNumber(js_item)) {
        *value = cJSON_GetNumberValue(js_item);
        return 0;
    } else {
        return -1;
    }
}

int device_get_item_string(device_handle_t handle, const char *key, char **value)
{
    cJSON *root = JSON_HANDLE(handle);
    if (NULL == root || NULL == key || NULL == value) {
        return -1;
    }
    cJSON *js_item = cJSON_GetObjectItemCaseSensitive(root, key);
    char *str_value = cJSON_GetStringValue(js_item);
    if (!str_value) {
        return -1;
    }
    *value = (char *)calloc(strlen(str_value) + 1, 1);
    if (!*value) {
        return -1;
    }
    memcpy(*value, str_value, strlen(str_value));
    return 0;
}

int device_set_item_int(device_handle_t handle, const char *key, int value) {
    cJSON *root = JSON_HANDLE(handle);
    if (NULL == root) {
        return -1;
    }
    cJSON *new_item = cJSON_CreateNumber(value);
    if (NULL == new_item) {
        return -1;
    }
    cJSON *item = cJSON_GetObjectItem(root, key);
    cJSON_bool ret = false;
    if (item) {
        ret = cJSON_ReplaceItemInObjectCaseSensitive(root, key, new_item);
    } else {
        ret = cJSON_AddItemToObject(root, key, new_item);
    }
    return (ret ? 0 : -1);
}

int device_set_item_double(device_handle_t handle, const char *key, double value) {
    cJSON *root = JSON_HANDLE(handle);
    if (NULL == root) {
        return -1;
    }
    cJSON *new_item = cJSON_CreateNumber(value);
    if (NULL == new_item) {
        return -1;
    }
    cJSON *item = cJSON_GetObjectItem(root, key);
    cJSON_bool ret = false;
    if (item) {
        ret = cJSON_ReplaceItemInObjectCaseSensitive(root, key, new_item);
    } else {
        ret = cJSON_AddItemToObject(root, key, new_item);
    }
    return (ret ? 0 : -1);
}

int device_set_item_string(device_handle_t handle, const char *key, const char *value) {
    cJSON *root = JSON_HANDLE(handle);
    if (NULL == root) {
        return -1;
    }
    cJSON *new_item = cJSON_CreateString(value);
    if (NULL == new_item) {
        return -1;
    }
    cJSON *item = cJSON_GetObjectItem(root, key);
    cJSON_bool ret = false;
    if (item) {
        ret = cJSON_ReplaceItemInObjectCaseSensitive(root, key, new_item);
    } else {
        ret = cJSON_AddItemToObject(root, key, new_item);
    }
    return (ret ? 0 : -1);
}