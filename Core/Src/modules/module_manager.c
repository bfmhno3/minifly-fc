#include "modules/module_manager.h"
#include "bsp_module.h"

#include "FreeRTOS.h"
#include "task.h"

#define MODULE_DEBOUNCE_THRESHOLD 3
#define MODULE_DETECT_PERIOD_MS   500
#define MODULE_ADC_TOLERANCE      50

typedef struct {
    uint16_t adc_center;
    bsp_module_id_t id;
} module_adc_entry_t;

typedef struct {
    bsp_module_id_t id;
    void (*init)(void);
    void (*deinit)(void);
} module_handler_entry_t;

static bsp_module_id_t active_module = BSP_MODULE_NONE;
static bsp_module_id_t detected_module = BSP_MODULE_NONE;
static uint8_t debounce_cnt = 0;

static const module_adc_entry_t adc_table[] = {
    { 2048, BSP_MODULE_LED_RING },
    { 4095, BSP_MODULE_WIFI_CAMERA },
    { 2815, BSP_MODULE_OPTICAL_FLOW },
    { 1280, BSP_MODULE_RESERVED_1 },
};

static void module_handler_none_init(void)         { }
static void module_handler_none_deinit(void)       { }
static void module_handler_led_ring_init(void)     { /* TODO: ledring_module_init() */ }
static void module_handler_led_ring_deinit(void)   { /* TODO: ledring_module_deinit() */ }
static void module_handler_wifi_init(void)         { /* TODO: wifi_module_init() */ }
static void module_handler_wifi_deinit(void)       { /* TODO: wifi_module_deinit() */ }
static void module_handler_optical_flow_init(void) { /* TODO: optical_flow_module_init() */ }
static void module_handler_optical_flow_deinit(void) { /* TODO: optical_flow_module_deinit() */ }
static void module_handler_reserved_init(void)     { }
static void module_handler_reserved_deinit(void)   { }

static const module_handler_entry_t handler_table[] = {
    { BSP_MODULE_NONE,         module_handler_none_init,         module_handler_none_deinit },
    { BSP_MODULE_LED_RING,     module_handler_led_ring_init,     module_handler_led_ring_deinit },
    { BSP_MODULE_WIFI_CAMERA,  module_handler_wifi_init,         module_handler_wifi_deinit },
    { BSP_MODULE_OPTICAL_FLOW, module_handler_optical_flow_init, module_handler_optical_flow_deinit },
    { BSP_MODULE_RESERVED_1,   module_handler_reserved_init,     module_handler_reserved_deinit },
};

static uint16_t my_abs_u16(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static bsp_module_id_t module_adc_match(uint16_t raw)
{
    for (uint32_t i = 0; i < sizeof(adc_table) / sizeof(adc_table[0]); i++) {
        if (my_abs_u16(raw, adc_table[i].adc_center) <= MODULE_ADC_TOLERANCE) {
            return adc_table[i].id;
        }
    }
    return BSP_MODULE_NONE;
}

static const module_handler_entry_t *module_find_handler(bsp_module_id_t id)
{
    for (uint32_t i = 0; i < sizeof(handler_table) / sizeof(handler_table[0]); i++) {
        if (handler_table[i].id == id) {
            return &handler_table[i];
        }
    }
    return &handler_table[0]; /* BSP_MODULE_NONE */
}

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
