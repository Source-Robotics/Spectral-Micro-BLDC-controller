/// @file cogcal.cpp
/// @brief Anti-cogging identification on the drive, see cogcal.h. The same in both firmwares.
#include <Arduino.h>
#include <string.h>
#include "cogcal.h"
#include "cogging.h"
#include "scope.h"
#include "common.h"
#include "constants.h"
#include "EEPROM.h"
#include "motor_control.h"
#include "qfplib-m3.h"
#include "encal.h"

#define CC_SETTLE_MS 700
#define CC_EMPTY INT32_MIN
#define CC_BINS SCOPE_HIST_BINS

volatile uint16_t cogcal_speed = 1500;
volatile int32_t cogcal_ripple_x10 = 0;
volatile int32_t cogcal_h1_before_x10 = 0, cogcal_h1_after_x10 = 0;
static uint8_t cc_trim = 0;            // 1: the run before the zero trim, 2: the run after it
static float cc_h1_a = 0.0f, cc_h1_b = 0.0f; // the last fit's 1st harmonic, mA (cos and sin of the cogging angle)
static int32_t cc_d1 = 0, cc_d2 = 0;   // the zero errors the trim removed, counts
static volatile uint8_t cc_state = COGCAL_IDLE;
static bool cc_save = false;
static bool cc_recording = false;
static uint32_t cc_t0 = 0;
static int32_t cc_cmd = 0;
static int32_t *cc_fwd = nullptr; // scratch (the capture buffer): mA x 16 per bin, CC_EMPTY where no sample
static int32_t *cc_bwd = nullptr;

uint8_t Cogcal_state()
{
  return cc_state;
}

static bool cc_running()
{
  return cc_state == COGCAL_FORWARD || cc_state == COGCAL_BACKWARD;
}

/// Velocity mode at v ticks/s, as a velocity frame from a host would set it.
static void cc_velocity(int32_t v)
{
  const uint32_t irq = __get_PRIMASK();
  __disable_irq();
  PID.Velocity_setpoint = v;
  PID.Feedforward_current = 0;
  Motion_wake();
  if (controller.Controller_mode != 1 && controller.Controller_mode != 2)
    PID.V_errSum = 0;
  controller.Controller_mode = 2;
  __set_PRIMASK(irq);
  cc_cmd = v;
}

/// Hold the position the shaft is at (safer than idle on a joint under gravity).
static void cc_hold()
{
  const uint32_t irq = __get_PRIMASK();
  __disable_irq();
  PID.Position_setpoint = controller.Position_Ticks;
  PID.Feedforward_speed = 0;
  PID.Feedforward_current = 0;
  controller.Controller_mode = 1;
  __set_PRIMASK(irq);
}

void Cogcal_start(bool save, bool trim)
{
  if (cc_running())
    return;
  if (controller.Error || controller.wrong_product || controller.Calibrated == 0)
  {
    cc_state = COGCAL_FAILED_ERROR;
    return;
  }
  int32_t *s = Scope_scratch();
  cc_fwd = s;
  cc_bwd = s + CC_BINS;
  cogging.mode = COGGING_MODE_OFF; // identify the motor as it is
  cc_save = save;
  cc_trim = trim ? 1 : 0;
  cc_recording = false;
  cc_t0 = millis();
  cc_state = COGCAL_FORWARD;
  cc_velocity(cogcal_speed);
}

void Cogcal_abort()
{
  if (!cc_running())
    return;
  Scope_hist_end();
  cc_hold();
  cc_state = COGCAL_FAILED_ABORTED;
}

/// The histogram's mean per bin, mA x 16. False if fewer than half the bins have samples (the shaft stuck).
static bool cc_store(int32_t *dst)
{
  int filled = 0;
  for (int k = 0; k < CC_BINS; k++)
  {
    int32_t v;
    if (Scope_hist_mean16(k, &v))
    {
      dst[k] = v;
      filled++;
    }
    else
    {
      dst[k] = CC_EMPTY;
    }
  }
  return filled >= CC_BINS / 2;
}

/// Forward and backward each minus its own mean, averaged; then the harmonics and the table into `cogging`.
static void cc_fit()
{
  int64_t sf = 0, sb = 0;
  int nf = 0, nb = 0;
  for (int k = 0; k < CC_BINS; k++)
  {
    if (cc_fwd[k] != CC_EMPTY) { sf += cc_fwd[k]; nf++; }
    if (cc_bwd[k] != CC_EMPTY) { sb += cc_bwd[k]; nb++; }
  }
  const int32_t mf = nf ? (int32_t)(sf / nf) : 0;
  const int32_t mb = nb ? (int32_t)(sb / nb) : 0;
  int32_t *c = cc_fwd; // in place
  int64_t ss = 0;
  for (int k = 0; k < CC_BINS; k++)
  {
    const bool hf = cc_fwd[k] != CC_EMPTY, hb = cc_bwd[k] != CC_EMPTY;
    const int32_t f = hf ? cc_fwd[k] - mf : 0, b = hb ? cc_bwd[k] - mb : 0;
    c[k] = (hf && hb) ? (f + b) / 2 : (hf ? f : (hb ? b : 0));
    ss += (int64_t)c[k] * c[k];
  }
  cogcal_ripple_x10 = (int32_t)(qfp_fsqrt((float)(ss / CC_BINS)) * 10.0f / 16.0f + 0.5f);

  // The 8 largest of harmonics 1-32: a = 2/N sum c cos(h phi), b = 2/N sum c sin(h phi) at the bin centres
  // (angle 64 k + 32), corrected for the bin average (sinc), as the bench script does.
  uint8_t best_h[COGGING_HARMONICS] = {0};
  float best_a[COGGING_HARMONICS] = {0}, best_b[COGGING_HARMONICS] = {0}, best_p[COGGING_HARMONICS];
  for (int s = 0; s < COGGING_HARMONICS; s++)
    best_p[s] = -1.0f;
  for (uint32_t h = 1; h <= COGGING_MAX_HARMONIC; h++)
  {
    int64_t A = 0, B = 0;
    for (int k = 0; k < CC_BINS; k++)
    {
      const uint32_t ang = h * (uint32_t)(64 * k + 32);
      A += (int64_t)c[k] * Cogging_sin_q15(ang + 4096);
      B += (int64_t)c[k] * Cogging_sin_q15(ang);
    }
    const float x = 3.14159265f * (float)h / (float)CC_BINS;
    const float g = qfp_fsin(x) / x;
    const float scale = 2.0f / (float)CC_BINS / 32768.0f / 16.0f / g;
    const float a = (float)A * scale, b = (float)B * scale, p = a * a + b * b;
    if (h == 1)
    {
      cc_h1_a = a;
      cc_h1_b = b;
    }
    int worst = 0;
    for (int s = 1; s < COGGING_HARMONICS; s++)
      if (best_p[s] < best_p[worst])
        worst = s;
    if (p > best_p[worst])
    {
      best_p[worst] = p;
      best_h[worst] = (uint8_t)h;
      best_a[worst] = a;
      best_b[worst] = b;
    }
  }
  const int32_t limit = (int32_t)(PID.Iq_current_limit / 2.0f);
  auto clamp = [limit](float v) -> int16_t {
    int32_t r = (int32_t)(v + (v >= 0 ? 0.5f : -0.5f));
    if (r > limit) r = limit;
    if (r < -limit) r = -limit;
    return (int16_t)r;
  };
  for (int s = 0; s < COGGING_HARMONICS; s++)
  {
    cogging.harmonic[s] = 0;
    cogging.a[s] = clamp(best_a[s]);
    cogging.b[s] = clamp(best_b[s]);
    cogging.harmonic[s] = best_p[s] >= 0.0f ? best_h[s] : 0;
  }
  // Table entry i sits at angle 64 i, between bins i-1 and i.
  for (int i = 0; i < COGGING_TABLE_LEN; i++)
    cogging.table[i] = clamp((float)(c[(i + CC_BINS - 1) % CC_BINS] + c[i]) / 32.0f);
  cogging.scale = 100;
  cogging.mode = COGGING_MODE_HARMONICS;
}

void Cogcal_run()
{
  if (!cc_running())
    return;
  controller.watchdog_reset = 1; // the drive is moving on its own: keep the CAN watchdog fed
  if (controller.Error)
  {
    Scope_hist_end();
    cc_state = COGCAL_FAILED_ERROR;
    return;
  }
  if (controller.Controller_mode != 2 || PID.Velocity_setpoint != cc_cmd)
  {
    Scope_hist_end(); // a host took over: leave the drive to it
    cc_state = COGCAL_FAILED_ABORTED;
    return;
  }
  const uint32_t now = millis();
  if (!cc_recording)
  {
    if (now - cc_t0 >= CC_SETTLE_MS)
    {
      Scope_hist_begin(HIST_X_ELECTRICAL, SIGNAL_IQ_SETPOINT_LOOP);
      cc_recording = true;
      cc_t0 = now;
    }
    return;
  }
  const uint32_t record_ms = 16384UL * 1000UL / (cogcal_speed < 100 ? 100 : cogcal_speed); // one motor turn
  if (now - cc_t0 < record_ms)
    return;
  Scope_hist_end();
  if (cc_state == COGCAL_FORWARD)
  {
    if (!cc_store(cc_fwd))
    {
      cc_hold();
      cc_state = COGCAL_FAILED_STUCK;
      return;
    }
    cc_recording = false;
    cc_t0 = now;
    cc_state = COGCAL_BACKWARD;
    cc_velocity(-(int32_t)cogcal_speed);
    return;
  }
  if (!cc_store(cc_bwd))
  {
    cc_hold();
    cc_state = COGCAL_FAILED_STUCK;
    return;
  }
  cc_hold();
  cc_fit();
  const float h1 = qfp_fsqrt(cc_h1_a * cc_h1_a + cc_h1_b * cc_h1_b);
  if (cc_trim == 1)
  {
    // The 1st harmonic is the current-sense zero error (bench 2026-11-08): correct the zeros, identify again.
    cogcal_h1_before_x10 = (int32_t)(h1 * 10.0f + 0.5f);
    cogcal_h1_after_x10 = cogcal_h1_before_x10;
    int32_t d1 = 0, d2 = 0;
    if (h1 >= COGCAL_TRIM_MIN_MA && Sense_trim_from_h1(cc_h1_a, cc_h1_b, &d1, &d2) && (d1 != 0 || d2 != 0))
    {
      if (d1 > COGCAL_TRIM_MAX) d1 = COGCAL_TRIM_MAX;
      if (d1 < -COGCAL_TRIM_MAX) d1 = -COGCAL_TRIM_MAX;
      if (d2 > COGCAL_TRIM_MAX) d2 = COGCAL_TRIM_MAX;
      if (d2 < -COGCAL_TRIM_MAX) d2 = -COGCAL_TRIM_MAX;
      cc_d1 = d1;
      cc_d2 = d2;
      Sense_trim_add(-d1, -d2);
      cc_trim = 2;
      cogging.mode = COGGING_MODE_OFF;
      cc_recording = false;
      cc_t0 = millis();
      cc_state = COGCAL_FORWARD;
      cc_velocity(cogcal_speed);
      return;
    }
  }
  else if (cc_trim == 2)
  {
    cogcal_h1_after_x10 = (int32_t)(h1 * 10.0f + 0.5f);
    if (cogcal_h1_after_x10 > 20 && cogcal_h1_after_x10 * 10 > cogcal_h1_before_x10 * 7)
    {
      Sense_trim_add(cc_d1, cc_d2); // it did not drop by 30 %: the map does not hold here, the zeros go back
      cogging.mode = COGGING_MODE_OFF; // and this set was found with the moved zeros: identify once more on the
      cc_trim = 3;                     // zeros as measured, so a set is on in the end (batch 20)
      cc_recording = false;
      cc_t0 = millis();
      cc_state = COGCAL_FORWARD;
      cc_velocity(cogcal_speed);
      return;
    }
  }
  const bool no_trim = (cc_trim == 3);
  cc_trim = 0;
  cc_state = no_trim ? COGCAL_DONE_NO_TRIM : COGCAL_DONE;
  if (cc_save)
    Write_config(); // the whole configuration: the anti-cogging and encoder blocks (with the trims) included
}

/// @brief The set was switched off because it no longer fits the encoder (a table measured on its own, encal.cpp):
/// the state reads COGCAL_STALE until the next identification (batch 18).
void Cogcal_mark_stale()
{
  if (!cc_running())
    cc_state = COGCAL_STALE;
}
