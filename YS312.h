/* YS312.h — SENBA YS312 PIR sensor driver for EFR32xG28
 *
 * Wiring (BRD2705A board):
 *   YS312 pin 1 (VDD)  → VMCU (3.3V)
 *   YS312 pin 2 (DOCI) → C01  (PC01, breakout pad)
 *   YS312 pin 3 (VSS)  → GND
 */
#ifndef YS312_H
#define YS312_H

#include <stdint.h>
#include <stdbool.h>

/* GPIO port and pin for DOCI line */
#define YS312_PORT   gpioPortC
#define YS312_PIN    1u

/* Result of a single sensor read */
typedef struct {
    int16_t value;  /* Signed 16-bit HPF output value           */
    bool    valid;  /* true = frame header and tail are correct  */
} YS312_Result;

/* Configure GPIO for DOCI communication */
void         YS312_Init(void);

/* Read one data frame from sensor (~150 µs + 19 bit periods) */
YS312_Result YS312_Read(void);

#endif /* YS312_H */
