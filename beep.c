/* beep.c — Passive buzzer driver for EFR32xG28
 *
 * Generates a square wave on GPIO pin by toggling at BEEP_PERIOD_US/2
 * intervals using sl_udelay_wait() busy-wait delay.
 */

#include "beep.h"
#include "sl_gpio.h"
#include "sl_udelay.h"
#include "sl_sleeptimer.h"

/* Initialize buzzer pin as push-pull output, initially LOW */
void Beep_Init(void)
{
    sl_gpio_set_pin_mode(&(sl_gpio_t){BEEP_PORT, BEEP_PIN},
                         SL_GPIO_MODE_PUSH_PULL, 0);
}

/* Generate buzzer tone for duration_ms milliseconds
 * Toggles pin at BEEP_PERIOD_US/2 intervals to produce square wave
 */
void Beep(uint32_t duration_ms)
{
    uint32_t t0 = sl_sleeptimer_tick_to_ms(sl_sleeptimer_get_tick_count());

    while (sl_sleeptimer_tick_to_ms(sl_sleeptimer_get_tick_count()) - t0 < duration_ms) {
        /* HIGH half-period */
        sl_gpio_set_pin(&(sl_gpio_t){BEEP_PORT, BEEP_PIN});
        sl_udelay_wait(BEEP_PERIOD_US / 2u);

        /* LOW half-period */
        sl_gpio_clear_pin(&(sl_gpio_t){BEEP_PORT, BEEP_PIN});
        sl_udelay_wait(BEEP_PERIOD_US / 2u);
    }

    /* Ensure pin is LOW after beep */
    sl_gpio_clear_pin(&(sl_gpio_t){BEEP_PORT, BEEP_PIN});
}
