
/* motion.h — Adaptive Swing-Based PIR motion detector
 *
 * Algorithm: detects bipolar swing characteristic of human motion.
 * A detection event requires BOTH a positive AND negative peak
 * within a rolling time window — single-sided spikes are ignored.
 *
 * Reference: AdaptiveSwingDetector (Python) ported to embedded C.
 */
#ifndef MOTION_H
#define MOTION_H
 
#include <stdint.h>
#include <stdbool.h>
 
/* ------------------------------------------------------------------ */
/*  Timing (sample rate = 1 sample per 200 ms → fs = 5 Hz)            */
/* ------------------------------------------------------------------ */
#define MOTION_FS_HZ            5u      /* Effective sample rate       */
#define MOTION_WINDOW_SEC       2u      /* Swing detection window (s)  */
#define MOTION_HOLD_SEC         2u      /* Hold time after detection   */
 
#define MOTION_WINDOW_LEN       (MOTION_FS_HZ * MOTION_WINDOW_SEC)  /* 10 */
#define MOTION_HOLD_COUNT       (MOTION_FS_HZ * MOTION_HOLD_SEC)    /* 10 */
 
/* Sensor warm-up time after power-on (ms) */
#define PIR_STARTUP_TIME        3000u
 
/* ------------------------------------------------------------------ */
/*  DC tracking (baseline)                                             */
/*  alpha_dc = 0.01 → scaled to integer: 1/100                        */
/* ------------------------------------------------------------------ */
#define DC_ALPHA_NUM            1
#define DC_ALPHA_DEN            100
 
/* ------------------------------------------------------------------ */
/*  Noise estimation                                                    */
/*  alpha_noise = 0.002 → scaled: 1/500                               */
/*  Updated only when |centered| < noise_level * 3                    */
/* ------------------------------------------------------------------ */
#define NOISE_ALPHA_NUM         1
#define NOISE_ALPHA_DEN         500
#define NOISE_GUARD_FACTOR      3       /* Don't update if signal > 3×noise */
 
/* ------------------------------------------------------------------ */
/*  Adaptive threshold                                                  */
/*  threshold = noise_level * sensitivity_ratio                        */
/*  sensitivity_ratio controlled by potentiometer (ADC)               */
/* ------------------------------------------------------------------ */
#define SENSITIVITY_RATIO_DEFAULT  10    
#define SENSITIVITY_RATIO_MIN      5   
#define SENSITIVITY_RATIO_MAX      15

/* Minimum absolute threshold (prevents triggering in perfect silence) */
#define THRESHOLD_ABS_MIN       8
 
/* ------------------------------------------------------------------ */
/*  State structure                                                     */
/* ------------------------------------------------------------------ */
typedef struct {
    /* DC tracking (baseline) — scaled × 256 for integer precision */
    int32_t  dc_level_scaled;
 
    /* Noise level estimate — scaled × 256 */
    int32_t  noise_level_scaled;
 
    /* Sensitivity ratio (set by potentiometer) */
    uint8_t  sensitivity_ratio;
 
    /* Rolling window of centered samples */
    int16_t  window[MOTION_WINDOW_LEN];
    uint8_t  win_idx;               /* Current write position (circular) */
 
    /* Hold timer */
    uint8_t  hold_counter;
 
    /* Output */
    bool     motion;
    int16_t  centered;              /* Last centered sample (for debug)  */
    int16_t  threshold;             /* Last computed threshold (debug)   */
} MotionDetector;
 
/* ------------------------------------------------------------------ */
/*  API                                                                 */
/* ------------------------------------------------------------------ */
 
/* Initialize with default values */
void MotionDetector_Init(MotionDetector *md);
 
/* Set sensitivity ratio from ADC reading (0..4095) */
void MotionDetector_SetSensitivity(MotionDetector *md, uint16_t adc_raw);
 
/* Process one sample, returns true when motion is active */
bool MotionDetector_Update(MotionDetector *md, int16_t pir_value);
 
#endif /* MOTION_H */