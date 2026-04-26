#ifndef __RADIOLINK_H
#define __RADIOLINK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RADIOLINK_FRAME_DATA_MAX 30

typedef struct {
	uint8_t msg_id;
	uint8_t data_len;
	uint8_t data[RADIOLINK_FRAME_DATA_MAX];
} radio_frame_t;

void radiolink_init(void);
void radiolink_task(void *arg);
bool radiolink_get_frame(radio_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __RADIOLINK_H */
