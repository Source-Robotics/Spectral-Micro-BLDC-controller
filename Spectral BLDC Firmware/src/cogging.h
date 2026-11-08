/// @file cogging.h
/// @brief Anti-cogging (2026-11): a feedforward current, a function of the electrical angle, added to the current
/// setpoint of the velocity and position loops. Two forms, to compare on the bench:
///   mode 1, harmonics: up to 8 terms a*cos(h*angle) + b*sin(h*angle), h 1-32, a and b in mA
///   mode 2, table: 256 values in mA over one electrical turn, linear interpolation between them
/// The angle is the electrical angle from the encoder alone, (Position_Raw * pole_pairs) mod 16384, without the
/// calibration's theta offset, so a recalibration does not move it. Saved in EEPROM block 3 by IN_SAVE_CONFIG.
/// This file is the same in the STEPFOC and Spectral firmwares.
#ifndef COGGING_H
#define COGGING_H

#include <stdint.h>

#define COGGING_HARMONICS 8      // harmonic slots
#define COGGING_MAX_HARMONIC 32  // highest harmonic of the electrical angle a slot may hold
#define COGGING_TABLE_LEN 256    // table values per electrical turn
#define COGGING_TABLE_SHIFT 6    // 16384 / 256 = 64 angle counts per table step
#define COGGING_BLOCK_VERSION 1
#define COGGING_MODE_OFF 0
#define COGGING_MODE_HARMONICS 1
#define COGGING_MODE_TABLE 2
#define COGGING_SCALE_MAX 200    // percent

/// The layout of EEPROM block 3 (constants.h COGGING_BLOCK_EEPROM): never reorder, add at the end with a new version.
typedef struct
{
    uint8_t version;                     // COGGING_BLOCK_VERSION
    uint8_t mode;                        // COGGING_MODE_*
    uint8_t scale;                       // percent of the stored values applied, 0-200 (100 = as stored)
    uint8_t reserved;
    uint8_t harmonic[COGGING_HARMONICS]; // 0 = slot unused
    int16_t a[COGGING_HARMONICS];        // mA, cosine term
    int16_t b[COGGING_HARMONICS];        // mA, sine term
    int16_t table[COGGING_TABLE_LEN];    // mA at angle i * 64
} CoggingBlock;

extern CoggingBlock cogging;
extern volatile int16_t cogging_ff_last; // the feedforward of the current loop, mA (0 when not applied)

uint32_t Cogging_angle();
int32_t Cogging_sin_q15(uint32_t angle);
int32_t Cogging_ff_mA();
void Cogging_defaults();
void Cogging_sanitize();
bool Cogging_has_set(uint8_t mode); // the set holds something for this mode (batch 18)

#endif
