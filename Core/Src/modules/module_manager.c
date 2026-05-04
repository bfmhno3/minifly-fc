// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Expansion module lifecycle manager implementation.
 *
 * @details
 * Uses an ADC resistor-ladder on the module connector to identify which
 * expansion module is attached.  A debounce counter filters out contact
 * bounce during hot-plug.  Once confirmed, the old module is de-initialized
 * and the new one is started via a handler dispatch table.
 *
 * Hardware dependencies:
 * - ADC1 channel for module identification (shared with bsp_module BSP).
 * - Module power pin controlled via bsp_module_power_set().
 */

#include "modules/module_manager.h"
#include "modules/ledring_module.h"
#include "modules/optical_flow_module.h"
#include "modules/wifi_module.h"
#include "bsp_module.h"

#include "FreeRTOS.h"
#include "task.h"

#define MODULE_DEBOUNCE_THRESHOLD 3     /**< consecutive matching reads before transition */
#define MODULE_DETECT_PERIOD_MS   500   /**< polling interval in milliseconds */
#define MODULE_ADC_TOLERANCE      50    /**< ADC counts, half-width of identification window */

/** @brief  Maps an ADC center value to a module ID. */
typedef struct {
    uint16_t adc_center;
    bsp_module_id_t id;
} module_adc_entry_t;

/** @brief  Maps a module ID to its init/deinit function pair. */
typedef struct {
    bsp_module_id_t id;
    void (*init)(void);
    void (*deinit)(void);
} module_handler_entry_t;

static bsp_module_id_t active_module = BSP_MODULE_NONE;
static bsp_module_id_t detected_module = BSP_MODULE_NONE;
static uint8_t debounce_cnt = 0;

/* --- ADC identification table ---
 * Each entry defines the expected ADC center value for a module.
 * Values come from the resistor divider on the module connector. */
static const module_adc_entry_t adc_table[] = {
    { 2048, BSP_MODULE_LED_RING },
    { 4095, BSP_MODULE_WIFI_CAMERA },
    { 2815, BSP_MODULE_OPTICAL_FLOW },
    { 1280, BSP_MODULE_RESERVED_1 },
};

/* --- per-module init/deinit thunks --- */

static void module_handler_none_init(void)         { }
static void module_handler_none_deinit(void)       { }
static void module_handler_led_ring_init(void)     { ledring_module_init(); }
static void module_handler_led_ring_deinit(void)   { ledring_module_deinit(); }
static void module_handler_wifi_init(void)         { wifi_module_init(); }
static void module_handler_wifi_deinit(void)       { wifi_module_deinit(); }
static void module_handler_optical_flow_init(void) { optical_flow_module_init(); }
static void module_handler_optical_flow_deinit(void) { optical_flow_module_deinit(); }
static void module_handler_reserved_init(void)     { }
static void module_handler_reserved_deinit(void)   { }

static const module_handler_entry_t handler_table[] = {
    { BSP_MODULE_NONE,         module_handler_none_init,         module_handler_none_deinit },
    { BSP_MODULE_LED_RING,     module_handler_led_ring_init,     module_handler_led_ring_deinit },
    { BSP_MODULE_WIFI_CAMERA,  module_handler_wifi_init,         module_handler_wifi_deinit },
    { BSP_MODULE_OPTICAL_FLOW, module_handler_optical_flow_init, module_handler_optical_flow_deinit },
    { BSP_MODULE_RESERVED_1,   module_handler_reserved_init,     module_handler_reserved_deinit },
};

/**
 * @brief  Unsigned absolute difference (avoids signed overflow risk).
 */
static uint16_t my_abs_u16(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

/**
 * @brief  Match a raw ADC reading to a module ID via the lookup table.
 *
 * @param[in] raw  12-bit ADC value from the module detect pin.
 *
 * @return Matched module ID, or BSP_MODULE_NONE if no entry is within tolerance.
 */
static bsp_module_id_t module_adc_match(uint16_t raw)
{
    for (uint32_t i = 0; i < sizeof(adc_table) / sizeof(adc_table[0]); i++) {
        if (my_abs_u16(raw, adc_table[i].adc_center) <= MODULE_ADC_TOLERANCE) {
            return adc_table[i].id;
        }
    }
    return BSP_MODULE_NONE;
}

/**
 * @brief  Look up the handler entry for a given module ID.
 *
 * Falls back to the BSP_MODULE_NONE handler if the ID is not found.
 */
static const module_handler_entry_t *module_find_handler(bsp_module_id_t id)
{
    for (uint32_t i = 0; i < sizeof(handler_table) / sizeof(handler_table[0]); i++) {
        if (handler_table[i].id == id) {
            return &handler_table[i];
        }
    }
    return &handler_table[0]; /* BSP_MODULE_NONE */
}

/**
 * @brief  Perform a module hot-swap transition.
 *
 * De-initializes the old module, cycles the shared power rail, then
 * initializes the new module.  The power cycle ensures a clean state
 * for the new module's hardware.
 */
static void module_apply_transition(bsp_module_id_t old_id, bsp_module_id_t new_id)
{
    const module_handler_entry_t *old_handler = module_find_handler(old_id);
    const module_handler_entry_t *new_handler = module_find_handler(new_id);

    old_handler->deinit();
    bsp_module_power_set(false);

    new_handler->init();
    if (new_id != BSP_MODULE_NONE) {
        bsp_module_power_set(true);
    }
}

void module_manager_init(void)
{
    active_module = BSP_MODULE_NONE;
    detected_module = BSP_MODULE_NONE;
    debounce_cnt = 0;
}

void module_manager_task(void *arg)
{
    (void)arg;

    for (;;) {
        uint16_t raw = bsp_module_detect_read_raw();
        bsp_module_id_t id = module_adc_match(raw);

        if (id == detected_module) {
            if (debounce_cnt < MODULE_DEBOUNCE_THRESHOLD) {
                debounce_cnt++;
            }
        } else {
            detected_module = id;
            debounce_cnt = 0;
        }

        if (debounce_cnt >= MODULE_DEBOUNCE_THRESHOLD && id != active_module) {
            module_apply_transition(active_module, id);
            active_module = id;
        }

        vTaskDelay(MODULE_DETECT_PERIOD_MS);
    }
}

bsp_module_id_t module_manager_get_active(void)
{
    return active_module;
}

void module_manager_power_enable(bool on)
{
    bsp_module_power_set(on);
}
