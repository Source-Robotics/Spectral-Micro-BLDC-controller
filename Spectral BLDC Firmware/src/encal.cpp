/// @file encal.cpp
/// @brief The encoder table on the drive, see encal.h. The same in both firmwares.
#include <Arduino.h>
#include <string.h>
#include "encal.h"
#include "cogcal.h"
#include "cogging.h"
#include "scope.h"
#include "common.h"
#include "constants.h"
#include "EEPROM.h"
#include "motor_control.h"
#include "qfplib-m3.h"

EncoderBlock encoder;
volatile int32_t encal_offset_x10 = 0;

// Main-loop state
static volatile uint8_t en_state = ENCAL_IDLE;
static bool en_save = false;
static uint8_t en_chain = 0;          // 1 the anti-cogging next, 2 and save all
static bool en_chain_failed = false;  // the encoder part of a combined run failed
static bool en_after_cal_pending = false;
static uint8_t en_after_what = AFTER_CAL_NOTHING; // what the pending run is
static bool en_skip_after_cal = false;             // #Calonly: the next successful calibration skips 56
static uint32_t en_t0 = 0;
static float en_saved_id = 0.0f;
static uint8_t en_saved_use = 0;
static int32_t *en_sum = nullptr; // scratch (the capture buffer): [0, 128) forward, [128, 256) backward, 1/16 count
static int32_t *en_cnt = nullptr; // samples per bin, the same layout

// Control-interrupt state
#define EN_OFF 0
#define EN_RAMP 1     // the current rises at the start angle
#define EN_FORWARD 2
#define EN_BACKWARD 3
#define EN_FINISHED 4 // both passes recorded: the field holds until the main loop has fitted
static volatile uint8_t en_phase = EN_OFF;
static volatile float en_theta = 0.0f;    // the commanded electrical angle, rad, [0, 2 pi)
static volatile float en_step = 0.0f;     // rad per loop (sign: direction)
static volatile float en_progress = 0.0f; // rad turned in this pass
static volatile float en_current = 0.0f;  // the d-axis current now, mA
static float en_target = 0.0f;            // mA
static float en_ramp_step = 0.0f;         // mA per loop
static float en_k = 0.0f;                 // 1/16 counts per electrical radian
static float en_span = 0.0f;              // one shaft turn, electrical rad
static volatile int32_t en_samples = 0;
static volatile int32_t en_bad = 0;       // samples more than 100° electrical off: the rotor is not following

uint8_t Encal_state()
{
  return en_state;
}

static bool en_running()
{
  return en_state == ENCAL_FORWARD || en_state == ENCAL_BACKWARD;
}

uint8_t Encal_chain_state(uint8_t cogcal_state)
{
  if (en_running() && en_chain)
    return COGCAL_ENCODER;
  if (en_chain_failed)
    return COGCAL_FAILED_ENCODER;
  return cogcal_state;
}

void Encoder_defaults()
{
  memset(&encoder, 0, sizeof(encoder));
  encoder.version = ENCODER_BLOCK_VERSION;
  encoder.after_cal = AFTER_CAL_DEFAULT;
}

void Encoder_sanitize()
{
  encoder.version = ENCODER_BLOCK_VERSION;
  if (encoder.after_cal > AFTER_CAL_BOTH)
    encoder.after_cal = AFTER_CAL_NOTHING;
  encoder.valid = encoder.valid ? 1 : 0;
  for (int k = 0; k < 2; k++)
    if (encoder.zero_trim[k] > ZERO_TRIM_MAX || encoder.zero_trim[k] < -ZERO_TRIM_MAX)
      encoder.zero_trim[k] = 0;
  encoder.use = (encoder.use && encoder.valid) ? 1 : 0;
}

uint16_t Encoder_correct(uint16_t raw)
{
  if (!encoder.use)
    return raw;
  const uint32_t i = raw >> 7, f = raw & 127;
  const int32_t t0 = encoder.table[i], t1 = encoder.table[(i + 1) & (ENCAL_TABLE_LEN - 1)];
  const int32_t corr16 = t0 + (((t1 - t0) * (int32_t)f) >> 7); // linear between the entries, 1/16 count
  return (uint16_t)(((int32_t)raw - ((corr16 + 8) >> 4)) & (CPR - 1));
}

/// Leave the open loop: restore the user's d-axis setpoint, hold the position the shaft is at.
static void en_hold()
{
  const uint32_t irq = __get_PRIMASK();
  __disable_irq();
  en_phase = EN_OFF;
  en_current = 0.0f;
  PID.Id_setpoint = en_saved_id;
  PID.Id_errSum = 0;
  PID.Iq_errSum = 0;
  PID.Position_setpoint = controller.Position_Ticks;
  PID.Feedforward_speed = 0;
  PID.Feedforward_current = 0;
  controller.Controller_mode = 1;
  __set_PRIMASK(irq);
}

void Encal_start(bool save, uint8_t then_cogcal)
{
  if (en_running() || Cogcal_state() == COGCAL_FORWARD || Cogcal_state() == COGCAL_BACKWARD)
    return;
  en_chain_failed = false;
  if (controller.Error || controller.wrong_product || controller.Calibrated == 0 || controller.pole_pairs <= 0)
  {
    en_state = ENCAL_FAILED_ERROR;
    if (then_cogcal)
      en_chain_failed = true;
    return;
  }
  int32_t *s = Scope_scratch();
  en_sum = s;
  en_cnt = s + 2 * ENCAL_TABLE_LEN;
  memset(s, 0, 4 * ENCAL_TABLE_LEN * sizeof(int32_t));
  en_save = save;
  en_chain = then_cogcal;
  en_saved_id = PID.Id_setpoint;
  en_saved_use = encoder.use;
  en_k = (float)CPR * 16.0f / (PI2 * (float)controller.pole_pairs);
  en_span = PI2 * (float)controller.pole_pairs;
  en_target = 0.8f * PID.Iq_current_limit;
  en_ramp_step = en_target / ((float)ENCAL_RAMP_MS * (float)LOOP_FREQ / 1000.0f);
  en_samples = 0;
  en_bad = 0;
  en_t0 = millis();
  en_state = ENCAL_FORWARD;

  const uint32_t irq = __get_PRIMASK();
  __disable_irq();
  encoder.use = 0; // measure the raw encoder; the angle below starts from the raw reading
  en_theta = controller.Electric_Angle;
  en_step = (float)ENCAL_TICKS_PER_S * PI2 * (float)controller.pole_pairs / (float)CPR / (float)LOOP_FREQ;
  en_progress = 0.0f;
  en_current = 0.0f;
  en_phase = EN_RAMP;
  PID.Id_errSum = 0;
  PID.Iq_errSum = 0;
  PID.Iq_setpoint = 0;
  Motion_wake();
  controller.Controller_mode = MODE_ENCAL;
  __set_PRIMASK(irq);
}

void Encal_abort()
{
  if (!en_running())
    return;
  encoder.use = en_saved_use;
  en_hold();
  en_state = ENCAL_FAILED_ABORTED;
  if (en_chain)
    en_chain_failed = true;
}

void Encal_step()
{
  if (en_phase == EN_OFF) // not measuring (an error or a host ended it): never drive a stale current
  {
    en_current = 0.0f;
    controller.Controller_mode = 0;
    return;
  }
  if (en_phase == EN_RAMP)
  {
    en_current += en_ramp_step;
    if (en_current >= en_target)
    {
      en_current = en_target;
      en_phase = EN_FORWARD;
    }
  }
  else if (en_phase == EN_FORWARD || en_phase == EN_BACKWARD)
  {
    // One electrical turn to settle, then exactly one shaft turn recorded, then one more to spare.
    if (en_progress >= PI2 && en_progress < PI2 + en_span)
    {
      float e = controller.Electric_Angle - en_theta; // Collect_data ran on the raw count (use = 0)
      if (e > 3.14159265f)
        e -= PI2;
      else if (e < -3.14159265f)
        e += PI2;
      if (e > 1.75f || e < -1.75f)
        en_bad++;
      en_samples++;
      const int bin = (controller.Position_Raw >> 7) + (en_phase == EN_BACKWARD ? ENCAL_TABLE_LEN : 0);
      en_sum[bin] += (int32_t)(e * en_k);
      en_cnt[bin]++;
    }
    en_theta += en_step;
    if (en_theta >= PI2)
      en_theta -= PI2;
    else if (en_theta < 0.0f)
      en_theta += PI2;
    en_progress += (en_step >= 0.0f) ? en_step : -en_step;
    if (en_progress >= en_span + 2.0f * PI2)
    {
      if (en_phase == EN_FORWARD)
      {
        en_phase = EN_BACKWARD;
        en_step = -en_step;
        en_progress = 0.0f;
      }
      else
      {
        en_phase = EN_FINISHED;
      }
    }
  }
  // The current loop on the commanded angle: a d-axis current there pulls the rotor's d-axis to it.
  Foc_frame(en_theta);
  PID.Id_setpoint = en_current;
  PID.Iq_setpoint = 0;
  Torque_mode();
}

/// Both passes in, forward and backward averaged per bin; the harmonics 1-16 and the table. Returns ENCAL_*.
static uint8_t en_fit()
{
  int32_t *c = en_sum; // in place: [0, 128) becomes the error per bin, 1/16 count
  int64_t total = 0;
  for (int b = 0; b < ENCAL_TABLE_LEN; b++)
  {
    if (en_cnt[b] == 0 || en_cnt[b + ENCAL_TABLE_LEN] == 0)
      return ENCAL_FAILED_STUCK;
    const int32_t f = en_sum[b] / en_cnt[b], r = en_sum[b + ENCAL_TABLE_LEN] / en_cnt[b + ENCAL_TABLE_LEN];
    c[b] = (f + r) / 2;
    total += c[b];
  }
  if (en_samples == 0 || en_bad > en_samples / 20)
    return ENCAL_FAILED_STUCK;
  const int32_t mean = (int32_t)(total / ENCAL_TABLE_LEN);

  // Mechanical harmonics 1-16 at the bin centres (raw 128 b + 64), corrected for the bin average (sinc).
  float a[ENCAL_HARMONICS + 1], b2[ENCAL_HARMONICS + 1];
  for (uint32_t h = 1; h <= ENCAL_HARMONICS; h++)
  {
    int64_t A = 0, B = 0;
    for (int b = 0; b < ENCAL_TABLE_LEN; b++)
    {
      const uint32_t ang = h * (uint32_t)(128 * b + 64);
      A += (int64_t)(c[b] - mean) * Cogging_sin_q15(ang + 4096);
      B += (int64_t)(c[b] - mean) * Cogging_sin_q15(ang);
    }
    const float x = 3.14159265f * (float)h / (float)ENCAL_TABLE_LEN;
    const float g = qfp_fsin(x) / x;
    const float scale = 2.0f / (float)ENCAL_TABLE_LEN / 32768.0f / g;
    a[h] = (float)A * scale;
    b2[h] = (float)B * scale;
  }
  int16_t table[ENCAL_TABLE_LEN];
  int32_t lo = INT32_MAX, hi = INT32_MIN;
  for (int i = 0; i < ENCAL_TABLE_LEN; i++)
  {
    float v = 0.0f;
    for (uint32_t h = 1; h <= ENCAL_HARMONICS; h++)
    {
      const uint32_t ang = h * (uint32_t)(128 * i);
      v += (a[h] * (float)Cogging_sin_q15(ang + 4096) + b2[h] * (float)Cogging_sin_q15(ang)) / 32768.0f;
    }
    int32_t t = (int32_t)(v + (v >= 0.0f ? 0.5f : -0.5f));
    if (t > 32767)
      t = 32767;
    if (t < -32767)
      t = -32767;
    table[i] = (int16_t)t;
    if (t < lo)
      lo = t;
    if (t > hi)
      hi = t;
  }
  // The mean error is the electrical offset's: corrected angle = commanded angle needs the offset moved by -mean.
  const float shift = -(float)mean / en_k; // electrical rad
  encal_offset_x10 = (int32_t)(shift * 572.957795f + (shift >= 0.0f ? 0.5f : -0.5f));
  // Implausible: more than 4° of the shaft peak to peak, or an offset move of more than 60° electrical.
  if ((hi - lo) > (int32_t)(4.0f * 16.0f * (float)CPR / 360.0f) || shift > 1.0472f || shift < -1.0472f)
    return ENCAL_REJECTED;

  const uint32_t irq = __get_PRIMASK();
  __disable_irq();
  memcpy(encoder.table, table, sizeof(table));
  encoder.valid = 1;
  encoder.use = 1;
  float off = controller.theta_offset + shift;
  while (off >= PI2)
    off -= PI2;
  while (off < 0.0f)
    off += PI2;
  controller.theta_offset = off;
  __set_PRIMASK(irq);
  return ENCAL_DONE;
}

void Encal_after_calibration()
{
  if (en_skip_after_cal)
  {
    en_skip_after_cal = false; // #Calonly
    return;
  }
  if (encoder.after_cal != AFTER_CAL_NOTHING)
  {
    en_after_what = encoder.after_cal;
    en_after_cal_pending = true;
    en_t0 = millis();
  }
}

/// @brief The electrical offset alone was calibrated again (STEPFOC #Calangle, batch 20): the encoder table stays valid,
/// the anti-cogging set depends on the offset, so it is identified again (with the trim) when 56 is not 0.
void Encal_after_angle_calibration()
{
  if (encoder.after_cal != AFTER_CAL_NOTHING)
  {
    en_after_what = AFTER_CAL_COGGING;
    en_after_cal_pending = true;
    en_t0 = millis();
  }
}

void Encal_run()
{
  if (en_after_cal_pending)
  {
    if (millis() - en_t0 < 500 || controller.Controller_mode != 0)
      return;
    en_after_cal_pending = false;
    // and save everything when it succeeds (batch 20): a host that saved right after the calibration would
    // otherwise have missed the table and the set
    if (en_after_what == AFTER_CAL_COGGING)
      Cogcal_start(true, true);
    else if (en_after_what == AFTER_CAL_BOTH)
      Encal_start(false, 2);
    return;
  }
  if (!en_running())
    return;
  controller.watchdog_reset = 1; // the drive is moving on its own: keep the CAN watchdog fed
  if (controller.Error)
  {
    encoder.use = en_saved_use;
    en_phase = EN_OFF;
    en_current = 0.0f;
    PID.Id_setpoint = en_saved_id;
    en_state = ENCAL_FAILED_ERROR;
    if (en_chain)
      en_chain_failed = true;
    return;
  }
  if (controller.Controller_mode != MODE_ENCAL)
  {
    encoder.use = en_saved_use; // a host took over: leave the drive to it
    en_phase = EN_OFF;
    en_current = 0.0f;
    PID.Id_setpoint = en_saved_id;
    en_state = ENCAL_FAILED_ABORTED;
    if (en_chain)
      en_chain_failed = true;
    return;
  }
  const uint8_t phase = en_phase;
  if (phase == EN_BACKWARD)
    en_state = ENCAL_BACKWARD;
  if (phase != EN_FINISHED)
  {
    const uint32_t expected_ms = 2UL * (16384UL * 1000UL / ENCAL_TICKS_PER_S) + 10000UL;
    if (millis() - en_t0 > expected_ms)
    {
      encoder.use = en_saved_use;
      en_hold();
      en_state = ENCAL_FAILED_STUCK;
      if (en_chain)
        en_chain_failed = true;
    }
    return;
  }
  const uint8_t result = en_fit();
  if (result != ENCAL_DONE)
    encoder.use = en_saved_use;
  en_hold();
  en_state = result;
  if (result != ENCAL_DONE)
  {
    if (en_chain)
      en_chain_failed = true;
    return;
  }
  if (en_chain)
    Cogcal_start(en_chain == 2, true); // the anti-cogging (with the zero trim) on the corrected encoder; saves all if asked
  else
  {
    // a set from before was found on the uncorrected encoder: off until identified again (batch 18)
    if (cogging.mode != COGGING_MODE_OFF)
    {
      cogging.mode = COGGING_MODE_OFF;
      Cogcal_mark_stale();
    }
    if (en_save)
      Write_config(); // the whole configuration: the encoder block, the moved offset and the anti-cogging mode
  }
}

/// Peak to peak and the once-per-turn part of the table in force, 0.01° of the shaft.
static void en_results(int32_t *pp_x100, int32_t *once_x100)
{
  int32_t lo = INT32_MAX, hi = INT32_MIN;
  int64_t A = 0, B = 0;
  for (int i = 0; i < ENCAL_TABLE_LEN; i++)
  {
    const int32_t t = encoder.table[i];
    if (t < lo)
      lo = t;
    if (t > hi)
      hi = t;
    A += (int64_t)t * Cogging_sin_q15((uint32_t)(128 * i) + 4096);
    B += (int64_t)t * Cogging_sin_q15((uint32_t)(128 * i));
  }
  const float to_x100 = 36000.0f / 16.0f / (float)CPR; // 1/16 count -> 0.01°
  *pp_x100 = encoder.valid ? (int32_t)((float)(hi - lo) * to_x100 + 0.5f) : 0;
  const float a = (float)A * 2.0f / (float)ENCAL_TABLE_LEN / 32768.0f;
  const float b = (float)B * 2.0f / (float)ENCAL_TABLE_LEN / 32768.0f;
  *once_x100 = encoder.valid ? (int32_t)(qfp_fsqrt(a * a + b * b) * to_x100 + 0.5f) : 0;
}

bool Encal_param_get(uint8_t param, int32_t *value)
{
  int32_t pp, once;
  switch (param)
  {
  case PARAM_ENCAL:
    *value = en_state;
    return true;
  case PARAM_ENCAL_USE:
    *value = encoder.use;
    return true;
  case PARAM_ENCAL_PP:
    en_results(&pp, &once);
    *value = pp;
    return true;
  case PARAM_ENCAL_ONCE:
    en_results(&pp, &once);
    *value = once;
    return true;
  case PARAM_AFTER_CAL:
    *value = encoder.after_cal;
    return true;
  case PARAM_ENCAL_OFFSET:
    *value = encal_offset_x10;
    return true;
  case PARAM_ZERO_TRIM1:
    *value = encoder.zero_trim[0];
    return true;
  case PARAM_ZERO_TRIM2:
    *value = encoder.zero_trim[1];
    return true;
  case PARAM_THETA_OFFSET:
    *value = (int32_t)(controller.theta_offset * 572.957795f + 0.5f);
    return true;
  case PARAM_TRIM_H1_BEFORE:
    *value = cogcal_h1_before_x10;
    return true;
  case PARAM_TRIM_H1_AFTER:
    *value = cogcal_h1_after_x10;
    return true;
  default:
    return false;
  }
}

bool Encal_param_set(uint8_t param, int32_t value)
{
  switch (param)
  {
  case PARAM_ENCAL:
    if (value == 0)
      Encal_abort();
    else if (value == 1 || value == 2)
      Encal_start(value == 2, 0);
    else if (value == 3)
      Encal_start(false, 2);
    return true;
  case PARAM_ENCAL_USE:
    if (en_running())
      return false;
    encoder.use = (value == 1 && encoder.valid) ? 1 : 0;
    return true;
  case PARAM_ZERO_TRIM1:
  case PARAM_ZERO_TRIM2:
    if (value < -ZERO_TRIM_MAX || value > ZERO_TRIM_MAX)
      return false;
    if (param == PARAM_ZERO_TRIM1)
      Sense_trim_add(value - encoder.zero_trim[0], 0);
    else
      Sense_trim_add(0, value - encoder.zero_trim[1]);
    return true;
  case PARAM_AFTER_CAL:
    if (value < AFTER_CAL_NOTHING || value > AFTER_CAL_BOTH)
      return false;
    encoder.after_cal = (uint8_t)value;
    return true;
  default:
    return false;
  }
}

void Sense_trim_add(int32_t dz1, int32_t dz2)
{
  int32_t t1 = encoder.zero_trim[0] + dz1, t2 = encoder.zero_trim[1] + dz2;
  if (t1 > ZERO_TRIM_MAX) t1 = ZERO_TRIM_MAX;
  if (t1 < -ZERO_TRIM_MAX) t1 = -ZERO_TRIM_MAX;
  if (t2 > ZERO_TRIM_MAX) t2 = ZERO_TRIM_MAX;
  if (t2 < -ZERO_TRIM_MAX) t2 = -ZERO_TRIM_MAX;
  dz1 = t1 - encoder.zero_trim[0];
  dz2 = t2 - encoder.zero_trim[1];
  const uint32_t irq = __get_PRIMASK();
  __disable_irq();
  controller.sense1_offset += dz1; // the zeros in use, and the bridge-off ones parameter 46 switches back to
  controller.sense2_offset += dz2;
  controller.sense1_offset_off += dz1;
  controller.sense2_offset_off += dz2;
  encoder.zero_trim[0] = (int8_t)t1;
  encoder.zero_trim[1] = (int8_t)t2;
  __set_PRIMASK(irq);
}

/// @brief The encoder-table and anti-cogging part of parameter 79 (batch 20); communication_CAN.cpp adds the rest.
uint32_t Encal_feature_bits()
{
  uint32_t f = 0;
  if (encoder.valid)
    f |= FEAT_ENCODER_MEASURED;
  if (encoder.use)
    f |= FEAT_ENCODER_IN_USE;
  if (Cogging_has_set(COGGING_MODE_HARMONICS) || Cogging_has_set(COGGING_MODE_TABLE))
    f |= FEAT_COGGING_SET;
  if (cogging.mode != COGGING_MODE_OFF)
    f |= FEAT_COGGING_ON;
  if (cogging.mode == COGGING_MODE_TABLE)
    f |= FEAT_COGGING_TABLE;
  if (controller.cogging_saved)
    f |= FEAT_COGGING_SAVED;
  if (encoder.zero_trim[0] != 0 || encoder.zero_trim[1] != 0)
    f |= FEAT_ZERO_TRIM;
  f |= (uint32_t)(encoder.after_cal & 3u) << FEAT_AFTER_CAL_SHIFT;
  if (en_running())
    f |= FEAT_RUN_ENCODER;
  if (Cogcal_state() == COGCAL_FORWARD || Cogcal_state() == COGCAL_BACKWARD)
    f |= FEAT_RUN_COGGING;
  if (en_after_cal_pending)
    f |= FEAT_RUN_PENDING;
  return f;
}

/// @brief #Calonly (true) or #Cal / #Calangle (false): whether the next successful calibration skips parameter 56.
void Encal_skip_next_after_calibration(bool skip)
{
  en_skip_after_cal = skip;
}

/// @brief What will follow the calibration that is running: AFTER_CAL_NOTHING for #Calonly, else parameter 56.
uint8_t Encal_after_calibration_planned()
{
  return en_skip_after_cal ? (uint8_t)AFTER_CAL_NOTHING : encoder.after_cal;
}
