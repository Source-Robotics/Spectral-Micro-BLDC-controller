/// @file encal.h
/// @brief The encoder table on the drive (2026-11, batch 9). The MT6816 reads the shaft angle wrong by a slowly
/// varying amount (the magnet is never exactly on the axis, the chip is not linear: ±0.75° typical), and the
/// commutation multiplies that by the pole pairs (37° electrical on a 50-pole-pair stepper). The drive turns the
/// field open loop - a d-axis current at a commanded angle, the current loop running on that angle - one shaft turn
/// forward and one back, records per 1/128 of a turn how far the encoder's electrical angle is from the commanded
/// one, averages both directions (friction flips sign, the encoder error does not), fits the mechanical harmonics
/// 1-16 (which leaves out the cogging pull at 50 and 200 per turn) and keeps a 128-entry table. The raw count is
/// corrected where it is read; the electrical offset is moved by the mean error so the commutation stays right.
/// Ben Katz's method (2017).
/// Started by #Encal (UART) or parameter 52 (CAN); parameter 43 or 52 = 3 runs it, then the anti-cogging
/// identification, and saves both. Parameter 56 runs either after a successful calibration. The same in both
/// firmwares; the angle-to-currents step is Foc_frame() in each motor_control.cpp.
#ifndef ENCAL_H
#define ENCAL_H

#include <stdint.h>

#define ENCAL_TABLE_LEN 128 // entries per shaft turn: entry i is the error at raw count 128 i, 1/16 count

typedef struct __attribute__((packed))
{
  uint8_t version;   // ENCODER_BLOCK_VERSION
  uint8_t use;       // 1: the raw count is corrected with the table (parameter 53)
  uint8_t after_cal; // AFTER_CAL_*: what runs when a calibration succeeds (parameter 56)
  uint8_t valid;     // 1: the table was measured
  int8_t zero_trim[2]; // current-sense zero trims, counts, added to the zeros measured at start (batch 11)
  int16_t gain_trim;   // reserved: a gain trim of current channel 2
  int16_t table[ENCAL_TABLE_LEN];
} EncoderBlock;

extern EncoderBlock encoder;
extern volatile int32_t encal_offset_x10; // the electrical offset the last run moved, 0.1° (parameter 57)

void Encal_start(bool save, uint8_t then_cogcal); // then_cogcal: 0 no, 1 the anti-cogging, 2 it and save all
void Encal_abort();
void Encal_run();               // main loop, every pass
void Encal_step();              // control interrupt, while Controller_mode is MODE_ENCAL
uint8_t Encal_state();          // ENCAL_*
uint8_t Encal_chain_state(uint8_t cogcal_state); // what parameter 43 reports while a combined run is on
uint16_t Encoder_correct(uint16_t raw);          // control interrupt, where the raw count is read
void Encoder_defaults();
void Encoder_sanitize();
void Encal_after_calibration(); // a calibration just succeeded: parameter 56 decides what follows
void Encal_after_angle_calibration(); // the offset alone (#Calangle): the anti-cogging follows if 56 is not 0
uint32_t Encal_feature_bits();      // the encoder and anti-cogging part of parameter 79
void Encal_skip_next_after_calibration(bool skip); // #Calonly
uint8_t Encal_after_calibration_planned(); // what follows the running calibration
bool Encal_param_get(uint8_t param, int32_t *value);
bool Encal_param_set(uint8_t param, int32_t value);
void Sense_trim_add(int32_t dz1, int32_t dz2); // move the zeros in use and the stored trims, counts

#endif
