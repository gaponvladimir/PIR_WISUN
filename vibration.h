/* vibration.h — Vibration detector using MPU-9265/6500 gyroscope + accel
 *
 * Algorithm: tracks a baseline noise level of frame-to-frame motion
 * energy (gyro delta + accel delta), then flags vibration when energy
 * exceeds an adaptive threshold (noise_level * sensitivity_ratio).
 *
 * Shares the same potentiometer as the PIR sensitivity control —
 * call VibrationDetector_SetSensitivity() with the same ADC reading.
 */
#ifndef VIBRATION_H
#define VIBRATION_H

#include <stdint.h>
#include <stdbool.h>
#include "mpu9265.h"

/* Noise level adaptation (alpha = 1/64 → ~12.8 sec time constant at 5Hz) */
#define VIB_NOISE_ALPHA_NUM      1
#define VIB_NOISE_ALPHA_DEN      64

/* Guard factor: don't update noise estimate when energy > noise * GUARD */
#define VIB_NOISE_GUARD_FACTOR   3

/* Sensitivity ratio range (same potentiometer as PIR) */
#define VIB_SENSITIVITY_DEFAULT  10
#define VIB_SENSITIVITY_MIN      6
#define VIB_SENSITIVITY_MAX      25

/* Minimum absolute threshold on energy metric */
#define VIB_THRESHOLD_ABS_MIN    150u

/* Confirmation: require N consecutive samples above threshold */
#define VIB_CONFIRM_COUNT        3u

/* Hold time after last detection (in samples, 200ms each → 1 sec) */
#define VIB_HOLD_COUNT           5u

typedef struct {
    /* Previous raw readings (for delta computation) */
    int16_t  prev_gx, prev_gy, prev_gz;
    int16_t  prev_ax, prev_ay, prev_az;
    bool     has_prev;

    /* Adaptive noise floor on energy metric, scaled x256 */
    int32_t  noise_level_scaled;

    /* Sensitivity ratio (from potentiometer) */
    uint8_t  sensitivity_ratio;

    /* Last computed values (debug) */
    uint32_t energy;
    uint32_t threshold;

    /* Confirmation / hold state machine */
    uint8_t  above_count;
    uint8_t  hold_counter;
    bool     vibration;
} VibrationDetector;

/* Initialize detector */
void VibrationDetector_Init(VibrationDetector *vd);

/* Set sensitivity ratio from ADC reading (0..4095) — shares pot with PIR */
void VibrationDetector_SetSensitivity(VibrationDetector *vd, uint16_t adc_raw);

/* Feed one IMU sample, returns true when vibration is active */
bool VibrationDetector_Update(VibrationDetector *vd, const MPU9265_Data *imu);

#endif /* VIBRATION_H */