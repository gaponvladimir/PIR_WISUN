/* YS312.c — SENBA YS312 PIR sensor driver for EFR32xG28
 *
 * Uses NOP-based delays for precise timing at 39 MHz.
 * Adjust DELAY_1US() NOP count if core clock changes.
 */

#include "YS312.h"
//#include "delay_nop.h"
#include "sl_sleeptimer.h"
#include "em_core.h"
#include "em_gpio.h"
#include "app_log.h"

/* ------------------------------------------------------------------ */
/*  Timing constants (µs)                                              */
/* ------------------------------------------------------------------ */
#define T_S_US   150u   /* Start signal HIGH duration (100us~5ms)      */
#define T_L_US     8u   /* LOW pulse width per bit (4us~8us)           */
#define T_H_US     8u   /* HIGH pulse width per bit (4us~8us)          */
#define T_B_US     8u   /* Stabilisation delay before sampling (4~8us) */
#define YS312_IDLE_TIMEOUT_MS  20u

/* Single NOP instruction */
#define NOP() __asm volatile ("nop")
 
/* ~1 µs at 39 MHz (adjust if needed) */
#define DELAY_1US()  do { \
    NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); \
    NOP(); NOP(); NOP(); \
} while(0)
 
/* Generic delay in microseconds (up to ~255 µs) */
static inline void delay_us(uint32_t us)
{
    while (us--) {
        DELAY_1US();
    }
}

/* ------------------------------------------------------------------ */
/*  Pin control helpers
 *
 *  Per datasheet: set output level FIRST, then switch to output mode.
 * ------------------------------------------------------------------ */

static inline void pin_drive_high(void)
{
    GPIO_PinOutSet(YS312_PORT, YS312_PIN);
    GPIO_PinModeSet(YS312_PORT, YS312_PIN, gpioModePushPull, 1);
}

static inline void pin_drive_low(void)
{
    GPIO_PinOutClear(YS312_PORT, YS312_PIN);
    GPIO_PinModeSet(YS312_PORT, YS312_PIN, gpioModePushPull, 0);
}

static inline void pin_release(unsigned int level)
{
    GPIO_PinModeSet(YS312_PORT, YS312_PIN, gpioModeInput, level);
}

static inline bool pin_read(void)
{
    return (GPIO_PinInGet(YS312_PORT, YS312_PIN) != 0u);
}

static inline uint32_t get_tick_ms(void)
{
    return sl_sleeptimer_tick_to_ms(sl_sleeptimer_get_tick_count());
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void YS312_Init(void)
{
    pin_release(0);
}

YS312_Result YS312_Read(void)
{
    YS312_Result result = { .value = 0, .valid = false };
    uint32_t raw = 0u;

    /* Step 1: wait for sensor to pull line HIGH */
    uint32_t t0 = get_tick_ms();
    while (!pin_read()) {
        if ((get_tick_ms() - t0) > YS312_IDLE_TIMEOUT_MS) {
            /* Nudge sensor */
            pin_drive_high();
            delay_us(10u);
            pin_release(1);

            t0 = get_tick_ms();
            while (!pin_read()) {
                if ((get_tick_ms() - t0) > YS312_IDLE_TIMEOUT_MS) {
                    app_log_error("YS312: sensor not responding\n\r");
                    return result;
                }
            }
            break;
        }
    }

    /* Step 2: line is HIGH (sensor drives it) — wait tS */
    delay_us(T_S_US);

    /* Step 3: clock out 19 bits MSB first */
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();

    for (int bit = 0; bit < 19; bit++) {
        /* Pull LOW for tL */
        pin_drive_low();
        delay_us(T_L_US);

        /* Drive HIGH for tH */
        pin_drive_high();
        delay_us(T_H_US);

        /* Release — sensor drives data bit */
        pin_release(1);

        /* Wait tB for stabilisation */
        delay_us(T_B_US);

        /* Sample the bit */
        raw <<= 1u;
        if (pin_read()) {
            raw |= 1u;
        }
    }

    /* Step 4: end-of-frame */
    pin_drive_low();
    delay_us(T_L_US);
    pin_release(0);

    CORE_EXIT_CRITICAL();

    /* Validate frame */
    bool b18_ok = (((raw >> 18u) & 1u) == 1u);
    bool b17_ok = (((raw >> 17u) & 1u) == 0u);
    bool b0_ok  = (((raw >>  0u) & 1u) == 0u);

    if (b18_ok && b17_ok && b0_ok) {
        uint16_t raw16 = (uint16_t)((raw >> 1u) & 0xFFFFu);
        result.value   = (int16_t)raw16;
        result.valid   = true;
    } else {
        app_log_debug("YS312: bad frame bits %u%u%u\n\r",
                      (unsigned int)(raw >> 18u) & 1u,
                      (unsigned int)(raw >> 17u) & 1u,
                      (unsigned int)(raw >>  0u) & 1u);
    }

    return result;
}