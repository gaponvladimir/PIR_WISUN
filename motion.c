/* motion.c — Adaptive Swing-Based PIR motion detector
 *
 * Ported from Python AdaptiveSwingDetector to embedded C.
 * Pure C, no platform dependencies.
 *
 * Key idea: motion is confirmed only when the signal swings BOTH
 * above +threshold AND below -threshold within the time window.
 * Single-sided spikes (electrical noise, heat sources) are ignored.
 */
#include "motion.h"
 
/* ------------------------------------------------------------------ */
/*  Initialization                                                      */
/* ------------------------------------------------------------------ */
void MotionDetector_Init(MotionDetector *md)
{
    md->dc_level_scaled    = 0;
    md->noise_level_scaled = 10 * 256; /* Start with noise = 10 counts */
    md->sensitivity_ratio  = SENSITIVITY_RATIO_DEFAULT;
    md->hold_counter       = 0;
    md->motion             = false;
    md->centered           = 0;
    md->threshold          = THRESHOLD_ABS_MIN;
    md->win_idx            = 0;
 
    for (uint8_t i = 0; i < MOTION_WINDOW_LEN; i++) {
        md->window[i] = 0;
    }
}
 
/* ------------------------------------------------------------------ */
/*  Set sensitivity ratio from potentiometer ADC reading               */
/* ------------------------------------------------------------------ */
void MotionDetector_SetSensitivity(MotionDetector *md, uint16_t adc_raw)
{
    /* Map 0..4095 → SENSITIVITY_RATIO_MAX..SENSITIVITY_RATIO_MIN
     * Inverted: ADC=0 (pot fully left) → most sensitive (ratio=MIN)
     *           ADC=4095 (pot fully right) → least sensitive (ratio=MAX)
     */
    md->sensitivity_ratio = (uint8_t)(SENSITIVITY_RATIO_MIN +
        ((uint32_t)adc_raw * (SENSITIVITY_RATIO_MAX - SENSITIVITY_RATIO_MIN))
        / 4095u);
}
 
/* ------------------------------------------------------------------ */
/*  Main update — call once per sample (every 200 ms)                  */
/* ------------------------------------------------------------------ */
bool MotionDetector_Update(MotionDetector *md, int16_t pir_value)
{
    /* ── Step 1: DC tracking (remove slow baseline drift) ─────────── */
    /* dc_level = dc_level * (1 - alpha) + pir_value * alpha
     * Integer form: dc_scaled = dc_scaled * 99/100 + pir_value * 256/100
     */
    md->dc_level_scaled = (md->dc_level_scaled * (DC_ALPHA_DEN - DC_ALPHA_NUM)
                          + (int32_t)pir_value * 256 * DC_ALPHA_NUM)
                          / DC_ALPHA_DEN;
 
    int16_t dc_level = (int16_t)(md->dc_level_scaled / 256);
    int16_t centered = pir_value - dc_level;
    md->centered = centered;
 
    /* ── Step 2: Noise level estimation ──────────────────────────────
     * Update only when signal is quiet (|centered| < noise * GUARD)
     * This prevents motion events from inflating the noise estimate.
     */
    int16_t noise_level = (int16_t)(md->noise_level_scaled / 256);
    int16_t abs_centered = centered < 0 ? -centered : centered;
 
    if (abs_centered < noise_level * NOISE_GUARD_FACTOR) {
        /* noise = noise * (1 - alpha) + |centered| * alpha */
        md->noise_level_scaled = (md->noise_level_scaled * (NOISE_ALPHA_DEN - NOISE_ALPHA_NUM)
                                  + (int32_t)abs_centered * 256 * NOISE_ALPHA_NUM)
                                  / NOISE_ALPHA_DEN;
        noise_level = (int16_t)(md->noise_level_scaled / 256);
    }
 
    /* ── Step 3: Compute adaptive threshold ──────────────────────────*/
    int16_t threshold = (int16_t)(noise_level * md->sensitivity_ratio);
    if (threshold < THRESHOLD_ABS_MIN) {
        threshold = THRESHOLD_ABS_MIN;
    }
    md->threshold = threshold;
 
    /* ── Step 4: Update rolling window (circular buffer) ─────────────*/
    md->window[md->win_idx] = centered;
    md->win_idx = (md->win_idx + 1) % MOTION_WINDOW_LEN;
 
    /* ── Step 5: Swing detection ─────────────────────────────────────
     * Motion confirmed only if window contains BOTH:
     *   - at least one sample > +threshold
     *   - at least one sample < -threshold
     */
    bool has_high = false;
    bool has_low  = false;
 
    for (uint8_t i = 0; i < MOTION_WINDOW_LEN; i++) {
        if (md->window[i] >  threshold) has_high = true;
        if (md->window[i] < -threshold) has_low  = true;
    }
 
    if (has_high && has_low) {
        /* Bipolar swing detected — reset hold timer */
        md->hold_counter = MOTION_HOLD_COUNT;
    }
 
    /* ── Step 6: Hold timer ──────────────────────────────────────────*/
    if (md->hold_counter > 0) {
        md->motion = true;
        md->hold_counter--;
    } else {
        md->motion = false;
    }
 
    return md->motion;
}
