/***************************************************************************//**
 * @file app.h
 * @brief Top level application functions
 ******************************************************************************/

#ifndef APP_H
#define APP_H

// Enable to use external sensivity regulator 
#define SENSIVITY_CONTROL_ENABLE

// Motion sensor enable
#define MOTION_ENABLE

// Vibration sensor enable
#define VIBRATION_ENABLE

// Wi_SUN connection  enable
#define WISUN_ENABLE

/* Delay after power-on before sensor init (ms) */
#define STARTUP_TIME_MS          1000u

/* Warmup period — detection output suppressed while baselines converge (ms) */
#define WARMUP_TIME_MS           60000u

/* Sensor polling period (ms) — effective sample rate = 5 Hz */
#define SENSOR_READING_PERIOD_MS 200u

/***************************************************************************//**
 * @brief Main application task.
 *
 * Called by the FreeRTOS thread created in app_init.c.
 * Performs hardware init, then runs the sensor polling loop.
 *
 * @param arg  Unused thread argument.
 ******************************************************************************/
void app_task(void *arg);

/***************************************************************************//**
 * @brief Application init — implemented in app_init.c.
 *
 * Creates the AppMain FreeRTOS thread which calls app_task().
 * Do not implement this function in app.c.
 ******************************************************************************/
void app_init(void);

/***************************************************************************//**
 * @brief App ticking function — not used in FreeRTOS project.
 ******************************************************************************/
void app_process_action(void);

#endif  /* APP_H */
