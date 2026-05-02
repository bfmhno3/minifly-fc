#ifndef MODULES_LEDRING_MODULE_H
#define MODULES_LEDRING_MODULE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LEDRING_EFFECT_OFF = 0,
    LEDRING_EFFECT_COLOR_TEST,
    LEDRING_EFFECT_ATTITUDE,
    LEDRING_EFFECT_GYRO,
    LEDRING_EFFECT_BLINK,
    LEDRING_EFFECT_FLASHLIGHT,
    LEDRING_EFFECT_BREATHING,
    LEDRING_EFFECT_RED_SPIN,
    LEDRING_EFFECT_COLOR_SPIN,
    LEDRING_EFFECT_DOUBLE_SPIN,
    LEDRING_EFFECT_COUNT,
} ledring_effect_t;

void ledring_module_init(void);
void ledring_module_deinit(void);
void ledring_module_set_effect(ledring_effect_t effect);
ledring_effect_t ledring_module_get_effect(void);

#ifdef __cplusplus
}
#endif

#endif /* MODULES_LEDRING_MODULE_H */
