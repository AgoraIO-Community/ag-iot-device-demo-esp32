#ifndef __INC_PACER_H__
#define __INC_PACER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
  uint32_t interval_ms;
  int64_t predict_next_time_ms;
} pacer_t;

void *pacer_create(uint32_t interval_ms);
void pacer_destroy(void *pacer);
void wait_for_next_pace(void *pacer);

#ifdef __cplusplus
}
#endif

#endif /* __INC_PACER_H__ */