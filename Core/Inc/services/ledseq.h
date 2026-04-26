#ifndef LEDSEQ_H
#define LEDSEQ_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_led.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LEDSEQ_WAIT_MS(x)  (x)
#define LEDSEQ_STOP        (-1)
#define LEDSEQ_LOOP        (-2)

struct ledseq_step {
    bool value;
    int  action;
};

enum ledseq_pattern {
    LEDSEQ_PATTERN_LOWBAT = 0,
    LEDSEQ_PATTERN_CHARGED,
    LEDSEQ_PATTERN_CHARGING,
    LEDSEQ_PATTERN_CALIBRATED,
    LEDSEQ_PATTERN_ALIVE,
    LEDSEQ_PATTERN_LINKUP,
    LEDSEQ_PATTERN_COUNT
};

#define LEDSEQ_LED_SYS       BSP_LED_GREEN_R
#define LEDSEQ_LED_LOWBAT    BSP_LED_RED_R
#define LEDSEQ_LED_CHG       BSP_LED_BLUE_L
#define LEDSEQ_LED_DATA_RX   BSP_LED_GREEN_L
#define LEDSEQ_LED_DATA_TX   BSP_LED_RED_L
#define LEDSEQ_LED_ERR1      BSP_LED_RED_L
#define LEDSEQ_LED_ERR2      BSP_LED_RED_R

void ledseq_init(void);
void ledseq_run(uint8_t target, uint8_t pattern);
void ledseq_stop(uint8_t target);
void ledseq_enable(bool enable);
bool ledseq_test(void);

#ifdef __cplusplus
}
#endif

#endif
