/// @file cogging.cpp
/// @brief Anti-cogging feedforward, see cogging.h. The same in the STEPFOC and Spectral firmwares.
#include <string.h>
#include "cogging.h"
#include "common.h"
#include "constants.h"

CoggingBlock cogging = {COGGING_BLOCK_VERSION, COGGING_MODE_OFF, 100, 0, {0}, {0}, {0}, {0}};
volatile int16_t cogging_ff_last = 0;

static_assert(sizeof(CoggingBlock) == 556, "CoggingBlock is the EEPROM block 3 layout");

/// sin(k * 90 deg / 64) in Q15, k = 0..64: a quarter wave
static const int16_t SIN_Q15_QUARTER[65] = {
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602, 6393, 7179, 7962, 8739, 9512,
    10278, 11039, 11793, 12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868,
    19519, 20159, 20787, 21403, 22005, 22594, 23170, 23731, 24279, 24811, 25329, 25832, 26319,
    26790, 27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571, 30852, 31113,
    31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757, 32767};

/// sin(angle * 2 pi / 16384) in Q15; linear interpolation, error under 4 / 32767. Integer only (no FPU here).
static inline int32_t Sin_q15(uint32_t angle)
{
  angle &= (CPR - 1);
  const uint32_t quadrant = angle >> 12;
  uint32_t r = angle & 4095;
  if (quadrant & 1)
    r = 4096 - r;
  const uint32_t i = r >> 6;
  int32_t v = SIN_Q15_QUARTER[i];
  if (i < 64)
    v += ((SIN_Q15_QUARTER[i + 1] - v) * (int32_t)(r & 63)) >> 6;
  return (quadrant & 2) ? -v : v;
}

/// @brief Sin_q15 for other files (cogcal.cpp).
int32_t Cogging_sin_q15(uint32_t angle)
{
  return Sin_q15(angle);
}

/// @brief The electrical angle from the encoder alone: 0-16383 per electrical turn, no theta offset.
uint32_t Cogging_angle()
{
  return ((uint32_t)controller.Position_Raw * (uint32_t)controller.pole_pairs) & (CPR - 1);
}

/// @brief The anti-cogging current for this loop in mA, 0 when off. Called by Velocity_mode() and Position_mode().
int32_t Cogging_ff_mA()
{
  int32_t ff = 0;
  const uint32_t angle = Cogging_angle();
  if (cogging.mode == COGGING_MODE_HARMONICS)
  {
    for (int s = 0; s < COGGING_HARMONICS; s++)
    {
      const uint32_t h = cogging.harmonic[s];
      if (h == 0 || (cogging.a[s] == 0 && cogging.b[s] == 0))
        continue; // an empty slot costs two table look-ups for nothing
      const uint32_t ha = h * angle;
      ff += ((int32_t)cogging.a[s] * Sin_q15(ha + 4096) + (int32_t)cogging.b[s] * Sin_q15(ha)) >> 15;
    }
  }
  else if (cogging.mode == COGGING_MODE_TABLE)
  {
    const uint32_t i = angle >> COGGING_TABLE_SHIFT;
    const int32_t f = (int32_t)(angle & ((1u << COGGING_TABLE_SHIFT) - 1));
    const int32_t t0 = cogging.table[i];
    const int32_t t1 = cogging.table[(i + 1) & (COGGING_TABLE_LEN - 1)];
    ff = t0 + (((t1 - t0) * f) >> COGGING_TABLE_SHIFT);
  }
  if (cogging.scale != 100)
    ff = ff * (int32_t)cogging.scale / 100;
  if (ff > 32767)
    ff = 32767;
  else if (ff < -32768)
    ff = -32768;
  cogging_ff_last = (int16_t)ff;
  return ff;
}

/// @brief Off, scale 100 %, every slot and value 0 (RAM only; Write_cogging_map() saves it).
void Cogging_defaults()
{
  cogging.mode = COGGING_MODE_OFF; // first: the control loop may be running
  memset(cogging.harmonic, 0, sizeof(cogging.harmonic));
  memset(cogging.a, 0, sizeof(cogging.a));
  memset(cogging.b, 0, sizeof(cogging.b));
  memset(cogging.table, 0, sizeof(cogging.table));
  cogging.version = COGGING_BLOCK_VERSION;
  cogging.scale = 100;
  cogging.reserved = 0;
}

/// @brief Clamp the block to what the firmware accepts.
void Cogging_sanitize()
{
  if (cogging.mode > COGGING_MODE_TABLE)
    cogging.mode = COGGING_MODE_OFF;
  if (cogging.scale > COGGING_SCALE_MAX)
    cogging.scale = 100;
  for (int s = 0; s < COGGING_HARMONICS; s++)
    if (cogging.harmonic[s] > COGGING_MAX_HARMONIC)
      cogging.harmonic[s] = 0;
  cogging.version = COGGING_BLOCK_VERSION;
}

/// @brief Whether the set holds anything for this mode (batch 18): a used harmonic slot with a or b, or a table value.
bool Cogging_has_set(uint8_t mode)
{
  if (mode == COGGING_MODE_HARMONICS)
  {
    for (int s = 0; s < COGGING_HARMONICS; s++)
      if (cogging.harmonic[s] != 0 && (cogging.a[s] != 0 || cogging.b[s] != 0))
        return true;
    return false;
  }
  if (mode == COGGING_MODE_TABLE)
  {
    for (int i = 0; i < COGGING_TABLE_LEN; i++)
      if (cogging.table[i] != 0)
        return true;
    return false;
  }
  return true;
}
