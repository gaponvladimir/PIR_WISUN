/* beep.h — PWM-based passive piezo buzzer driver
 *
 * Replaces GPIO toggle approach with hardware PWM (Simple PWM component).
 * The passive buzzer requires an external square wave at the desired
 * frequency — PWM duty cycle controls loudness, frequency controls pitch.
 */

#ifndef BEEP_H
#define BEEP_H

#include <stdint.h>

/* Default tone frequency in Hz — typical piezo resonance is 2-4 kHz */
#define BEEP_DEFAULT_FREQ_HZ   2000u

/* Default duty cycle in percent (0-100). 50% is standard for square wave. */
#define BEEP_DEFAULT_DUTY_PCT  50u

/***************************************************************************//**
 * @brief Initialize the PWM buzzer driver.
 *
 * Must be called once during app_task() init, after sl_simple_pwm_init()
 * has run (handled automatically via instance init in autogen).
 ******************************************************************************/
void Beep_Init(void);

/***************************************************************************//**
 * @brief Play a tone for a fixed duration, blocking.
 *
 * @param duration_ms  How long to sound the buzzer, in milliseconds.
 *                      Uses the default frequency and duty cycle.
 ******************************************************************************/
void Beep(uint32_t duration_ms);

/***************************************************************************//**
 * @brief Play a tone at a specific frequency for a fixed duration, blocking.
 *
 * @param freq_hz      Tone frequency in Hz (typical piezo range 1000-5000 Hz).
 * @param duration_ms  How long to sound the buzzer, in milliseconds.
 ******************************************************************************/
void Beep_Tone(uint32_t freq_hz, uint32_t duration_ms);

/***************************************************************************//**
 * @brief Start the buzzer continuously at a given frequency (non-blocking).
 *
 * @param freq_hz  Tone frequency in Hz.
 ******************************************************************************/
void Beep_Start(uint32_t freq_hz);

/***************************************************************************//**
 * @brief Stop the buzzer (sets duty cycle to 0).
 ******************************************************************************/
void Beep_Stop(void);

#endif  /* BEEP_H */
