// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  LED sequence animation engine implementation.
 *
 * @details
 * Each LED gets a FreeRTOS software timer.  When a pattern runs, the timer
 * callback (run_ledseq) steps through the pattern array, calling bsp_led_set
 * for on/off steps and rearming the timer with the step's delay.
 * Priority arbitration: the highest-priority (lowest index) non-stopped
 * pattern wins the LED.
 */

#include "services/ledseq.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

static const struct ledseq_step seq_lowbat[] = {
    { true, LEDSEQ_WAIT_MS(1000) },
    { false, LEDSEQ_LOOP },
};

static const struct ledseq_step seq_charged[] = {
    { true, LEDSEQ_WAIT_MS(1000) },
    { false, LEDSEQ_LOOP },
};

static const struct ledseq_step seq_charging[] = {
    { true, LEDSEQ_WAIT_MS(200) },
    { false, LEDSEQ_WAIT_MS(800) },
    { false, LEDSEQ_LOOP },
};

static const struct ledseq_step seq_calibrated[] = {
    { true, LEDSEQ_WAIT_MS(50) },
    { false, LEDSEQ_WAIT_MS(450) },
    { false, LEDSEQ_LOOP },
};

static const struct ledseq_step seq_alive[] = {
    { true, LEDSEQ_WAIT_MS(50) },
    { false, LEDSEQ_WAIT_MS(1950) },
    { false, LEDSEQ_LOOP },
};

static const struct ledseq_step seq_linkup[] = {
    { true, LEDSEQ_WAIT_MS(1) },
    { false, LEDSEQ_WAIT_MS(0) },
    { false, LEDSEQ_STOP },
};

static const struct ledseq_step *sequences[LEDSEQ_PATTERN_COUNT] = {
    seq_lowbat, seq_charged, seq_charging, seq_calibrated, seq_alive, seq_linkup,
};

static void update_active(uint8_t led);
static int get_prio(uint8_t pattern);
static void run_ledseq(TimerHandle_t xTimer);

static bool is_init;
static bool enabled = true;
static int active_seq[BSP_LED_COUNT];
static int state[BSP_LED_COUNT][LEDSEQ_PATTERN_COUNT];
static TimerHandle_t timers[BSP_LED_COUNT];
static SemaphoreHandle_t sem;

/** @brief  See ledseq.h */
void ledseq_init(void)
{
    if (is_init)
        return;

    for (int i = 0; i < BSP_LED_COUNT; i++) {
        active_seq[i] = LEDSEQ_STOP;
        for (int j = 0; j < LEDSEQ_PATTERN_COUNT; j++)
            state[i][j] = LEDSEQ_STOP;
    }

    for (int i = 0; i < BSP_LED_COUNT; i++)
        timers[i] = xTimerCreate("ledseq", 1000, pdFALSE, (void *)(uintptr_t)i, run_ledseq);

    sem = xSemaphoreCreateBinary();
    xSemaphoreGive(sem);

    is_init = true;
}

/** @brief  See ledseq.h */
void ledseq_run(uint8_t target, uint8_t pattern)
{
    int prio = get_prio(pattern);
    if (prio < 0)
        return;

    xSemaphoreTake(sem, portMAX_DELAY);
    state[target][prio] = 0;
    update_active(target);
    xSemaphoreGive(sem);

    if (active_seq[target] == prio)
        run_ledseq(timers[target]);
}

/** @brief  See ledseq.h */
void ledseq_stop(uint8_t target)
{
    xSemaphoreTake(sem, portMAX_DELAY);
    for (int i = 0; i < LEDSEQ_PATTERN_COUNT; i++)
        state[target][i] = LEDSEQ_STOP;
    update_active(target);
    xSemaphoreGive(sem);

    run_ledseq(timers[target]);
}

/** @brief  See ledseq.h */
void ledseq_enable(bool enable)
{
    enabled = enable;
}

/** @brief  See ledseq.h */
bool ledseq_test(void)
{
    ledseq_enable(true);
    return is_init;
}

static void run_ledseq(TimerHandle_t xTimer)
{
    bool leave = false;
    uint8_t led = (uint8_t)(uintptr_t)pvTimerGetTimerID(xTimer);

    if (!enabled)
        return;

    while (!leave) {
        int prio = active_seq[led];

        if (prio == LEDSEQ_STOP)
            return;

        const struct ledseq_step *step = &sequences[prio][state[led][prio]];
        state[led][prio]++;

        xSemaphoreTake(sem, portMAX_DELAY);
        switch (step->action) {
        case LEDSEQ_LOOP:
            state[led][prio] = 0;
            break;
        case LEDSEQ_STOP:
            state[led][prio] = LEDSEQ_STOP;
            update_active(led);
            break;
        default:
            bsp_led_set(led, step->value);
            if (step->action == 0)
                break;
            xTimerChangePeriod(xTimer, step->action, 0);
            xTimerStart(xTimer, 0);
            leave = true;
            break;
        }
        xSemaphoreGive(sem);
    }
}

static int get_prio(uint8_t pattern)
{
    if (pattern < LEDSEQ_PATTERN_COUNT)
        return (int)pattern;
    return -1;
}

static void update_active(uint8_t led)
{
    bsp_led_set(led, false);
    active_seq[led] = LEDSEQ_STOP;

    for (int prio = 0; prio < LEDSEQ_PATTERN_COUNT; prio++) {
        if (state[led][prio] != LEDSEQ_STOP) {
            active_seq[led] = prio;
            break;
        }
    }
}
