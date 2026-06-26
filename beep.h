/* beep.h — Passive buzzer driver for EFR32xG28
 *
 * Wiring (BRD2705A):
 *   Buzzer+ → PC02 (breakout pad C02)
 *   Buzzer- → GND
 */
#ifndef BEEP_H
#define BEEP_H

#include <stdint.h>

/* GPIO port and pin for buzzer */
#define BEEP_PORT   gpioPortC
#define BEEP_PIN    2u

/* Buzzer tone period in microseconds (1000 µs = 1 kHz) */
#define BEEP_PERIOD_US  1000u

/* Initialize buzzer GPIO */
void Beep_Init(void);

/* Generate buzzer tone for specified duration in milliseconds */
void Beep(uint32_t duration_ms);

#endif /* BEEP_H */
