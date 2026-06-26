/* vibration.c — Vibration detector using MPU-9265/6500 gyroscope + accel
 *
 * Energy metric = sum of squared frame-to-frame deltas across
 * gyro X/Y/Z and accel X/Y/Z. Gyro deltas dominate for rotational
 * shake; accel deltas catch linear taps/knocks.
 *
 * To keep the metric in a reasonable integer range, deltas are
 * right-shifted before squaring (gyro >>2, accel >>4) — gyro LSBs
 * are smaller in magnitude relative to noise than accel LSBs.
 */
#include "vibration.h"

/* ------------------------------------------------------------------ */
/*  Helpers                                                             */
/* ------------------------------------------------------------------ */
static inline int32_t iabs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* ------------------------------------------------------------------ */
/*  Initialization                                                      */
/* ------------------------------------------------------------------ */
void VibrationDetector_Init(VibrationDetector *vd)
{
    vd->has_prev           = false;
    vd->prev_gx = vd->prev_gy = vd->prev_gz = 0;
    vd->prev_ax = vd->prev_ay = vd->prev_az = 0;

    vd->noise_level_scaled = 20 * 256; /* Start with a small noise estimate */
    vd->sensitivity_ratio  = VIB_SENSITIVITY_DEFAULT;

    vd->energy        = 0;
    vd->threshold      = VIB_THRESHOLD_ABS_MIN;
    vd->above_count    = 0;
    vd->hold_counter    = 0;
    vd->vibration       = false;
}

/* ------------------------------------------------------------------ */
/*  Sensitivity from potentiometer                                      */
/* ------------------------------------------------------------------ */
void VibrationDetector_SetSensitivity(VibrationDetector *vd, uint16_t adc_raw)
{
    vd->sensitivity_ratio = (uint8_t)(VIB_SENSITIVITY_MIN +
        ((uint32_t)adc_raw * (VIB_SENSITIVITY_MAX - VIB_SENSITIVITY_MIN))
        / 4095u);
}

/* ------------------------------------------------------------------ */
/*  Main update                                                         */
/* ------------------------------------------------------------------ */
bool VibrationDetector_Update(VibrationDetector *vd, const MPU9265_Data *imu)
{
    if (!vd->has_prev) {
        /* First sample — just store baseline, no detection yet */
        vd->prev_gx = imu->gyro_x;  vd->prev_gy = imu->gyro_y;  vd->prev_gz = imu->gyro_z;
        vd->prev_ax = imu->accel_x; vd->prev_ay = imu->accel_y; vd->prev_az = imu->accel_z;
        vd->has_prev = true;
        return false;
    }

    /* ── Step 1: compute frame-to-frame deltas ────────────────────── */
    int32_t dgx = (int32_t)imu->gyro_x  - vd->prev_gx;
    int32_t dgy = (int32_t)imu->gyro_y  - vd->prev_gy;
    int32_t dgz = (int32_t)imu->gyro_z  - vd->prev_gz;
    int32_t dax = (int32_t)imu->accel_x - vd->prev_ax;
    int32_t day = (int32_t)imu->accel_y - vd->prev_ay;
    int32_t daz = (int32_t)imu->accel_z - vd->prev_az;

    vd->prev_gx = imu->gyro_x;  vd->prev_gy = imu->gyro_y;  vd->prev_gz = imu->gyro_z;
    vd->prev_ax = imu->accel_x; vd->prev_ay = imu->accel_y; vd->prev_az = imu->accel_z;

    /* ── Step 2: scale down to keep squares in 32-bit range ──────────
     * Gyro deltas can be up to ~65000 (full scale swing); shift right
     * by 4 bits. Accel deltas up to ~65000 too; shift right by 4 bits.
     */
    int32_t sgx = dgx >> 4, sgy = dgy >> 4, sgz = dgz >> 4;
    int32_t sax = dax >> 4, say = day >> 4, saz = daz >> 4;

    /* ── Step 3: energy = sum of squared scaled deltas ───────────────*/
    uint32_t energy = (uint32_t)(sgx * sgx + sgy * sgy + sgz * sgz
                                 + sax * sax + say * say + saz * saz);
    vd->energy = energy;

    /* ── Step 4: update noise floor (only when quiet) ────────────────*/
    int32_t noise_level = vd->noise_level_scaled / 256;

    if (energy < (uint32_t)noise_level * VIB_NOISE_GUARD_FACTOR) {
        vd->noise_level_scaled = (vd->noise_level_scaled * (VIB_NOISE_ALPHA_DEN - VIB_NOISE_ALPHA_NUM)
                                  + (int32_t)energy * 256 * VIB_NOISE_ALPHA_NUM)
                                  / VIB_NOISE_ALPHA_DEN;
        noise_level = vd->noise_level_scaled / 256;
    }

    /* ── Step 5: compute adaptive threshold ───────────────────────────*/
    uint32_t threshold = (uint32_t)noise_level * vd->sensitivity_ratio;
    if (threshold < VIB_THRESHOLD_ABS_MIN) {
        threshold = VIB_THRESHOLD_ABS_MIN;
    }
    vd->threshold = threshold;

    /* ── Step 6: confirmation counter ─────────────────────────────────*/
    if (energy > threshold) {
        if (vd->above_count < 255u) {
            vd->above_count++;
        }
    } else {
        vd->above_count = 0;
    }

    if (vd->above_count >= VIB_CONFIRM_COUNT) {
        vd->hold_counter = VIB_HOLD_COUNT;
    }

    /* ── Step 7: hold timer ────────────────────────────────────────── */
    if (vd->hold_counter > 0) {
        vd->vibration = true;
        vd->hold_counter--;
    } else {
        vd->vibration = false;
    }

    return vd->vibration;
}
