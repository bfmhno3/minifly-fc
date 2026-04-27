#ifndef __USBLINK_H
#define __USBLINK_H

#include <stdbool.h>
#include "comm/atkp.h"

#ifdef __cplusplus
extern "C" {
#endif

void usblink_init(void);
void usblink_rx_task(void *arg);
void usblink_tx_task(void *arg);
bool usblink_get_frame(atkp_frame_t *frame);
bool usblink_send_frame(atkp_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __USBLINK_H */
