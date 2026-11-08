/// @file scope.cpp
/// @brief Capture, histogram and the buffer read/write protocol, see scope.h and constants.h (IN_BUFFER_READ).
/// The same in the STEPFOC and Spectral firmwares.
#include <string.h>
#include "scope.h"
#include "cogging.h"
#include "common.h"
#include "constants.h"
#include "CAN.h"
#include "communication_CAN.h"
#include "cogcal.h"
#include "encal.h"

// --- capture: a ring of SCOPE_LEN samples per channel ---
static int16_t scope_buf[SCOPE_CHANNELS][SCOPE_LEN] __attribute__((aligned(4)));
static volatile uint8_t cap_source[SCOPE_CHANNELS] = {SIGNAL_VELOCITY, SIGNAL_IQ, SIGNAL_ELECTRICAL_ANGLE};
static volatile uint16_t cap_divider = 1;     // control loops per sample
static volatile uint8_t cap_mode = TRIGGER_STOP;
static volatile int16_t cap_level = 0;
static volatile uint16_t cap_pretrigger = 0;
static volatile uint8_t cap_state = CAPTURE_IDLE;
static volatile uint16_t cap_write = 0;       // ring position of the next sample
static volatile uint16_t cap_filled = 0;      // samples in the ring, up to SCOPE_LEN
static volatile uint16_t cap_after = 0;       // samples still to record after the trigger
static volatile uint16_t cap_trigger_pos = 0; // ring position of the trigger sample
static volatile bool cap_triggered = false;
static volatile uint16_t cap_tick = 0;
static volatile int16_t cap_prev_a = 0;
static volatile bool cap_prev_error = false;
static volatile bool cap_error_edge = false;

// --- histogram: the sum and count of one signal per angle bin ---
static int32_t hist_sum[SCOPE_HIST_BINS];
static uint16_t hist_count[SCOPE_HIST_BINS];
static volatile uint8_t hist_x = HIST_X_OFF;
static volatile uint8_t hist_y = SIGNAL_IQ_SETPOINT;
static volatile uint32_t hist_samples = 0;

// --- read-out in progress (main loop only) ---
static uint8_t out_kind = 0;
static uint16_t out_next = 0;
static uint16_t out_end = 0;

#define COMPILER_BARRIER() __asm volatile("" ::: "memory")

static inline int16_t Sat16(int32_t v)
{
  if (v > 32767)
    return 32767;
  if (v < -32768)
    return -32768;
  return (int16_t)v;
}

/// @brief One sample of a signal (constants.h SIGNAL_*), as int16.
static int16_t Scope_signal(uint8_t source)
{
  switch (source)
  {
  case SIGNAL_IQ:
    return Sat16((int32_t)FOC.Iq);
  case SIGNAL_ID:
    return Sat16((int32_t)FOC.Id);
  case SIGNAL_IQ_SETPOINT:
    return Sat16((int32_t)PID.Iq_setpoint);
  case SIGNAL_VELOCITY:
    return Sat16(controller.Velocity_Filter);
  case SIGNAL_VELOCITY_16:
    return Sat16(controller.Velocity_Filter / 16);
  case SIGNAL_POSITION:
    return (int16_t)(uint16_t)(controller.Position_Ticks & 0xFFFF);
  case SIGNAL_POSITION_ERROR:
    return Sat16(PID.Position_setpoint - controller.Position_Ticks);
  case SIGNAL_ELECTRICAL_ANGLE:
    return (int16_t)Cogging_angle();
  case SIGNAL_ENCODER:
    return (int16_t)controller.Position_Raw;
  case SIGNAL_VBUS:
    return Sat16(controller.VBUS_mV / 10);
  case SIGNAL_SENSE1:
    return Sat16(controller.Sense1_Raw);
  case SIGNAL_SENSE2:
    return Sat16(controller.Sense2_Raw);
  case SIGNAL_LOOP_TIME:
    return Sat16(controller.execution_time);
  case SIGNAL_COGGING_FF:
    return cogging_ff_last;
  case SIGNAL_IQ_SETPOINT_LOOP:
    return Sat16((int32_t)PID.Iq_setpoint - cogging_ff_last);
  case SIGNAL_UQ:
    return Sat16(PID.Uq_setpoint);
  case SIGNAL_UD:
    return Sat16((int32_t)FOC.Ud);
  case SIGNAL_UQ_APPLIED:
    return Sat16((int32_t)FOC.Uq);
  default:
    return 0;
  }
}

/// @brief At the end of every control loop: histogram, then capture. Integer work only.
void Scope_tick()
{
  const bool err = controller.Error;
  if (err && !cap_prev_error)
    cap_error_edge = true;
  cap_prev_error = err;

  const uint8_t hx = hist_x;
  if (hx != HIST_X_OFF)
  {
    const uint32_t angle = (hx == HIST_X_ELECTRICAL) ? Cogging_angle() : (uint32_t)controller.Position_Raw;
    const uint32_t bin = (angle & (CPR - 1)) >> SCOPE_HIST_SHIFT;
    hist_sum[bin] += Scope_signal(hist_y);
    hist_samples = hist_samples + 1;
    hist_count[bin]++;
    if (hist_count[bin] == 0xFFFF)
      hist_x = HIST_X_OFF; // a bin is full: stop, keep the data
  }

  const uint8_t st = cap_state;
  if (st == CAPTURE_ARMED || st == CAPTURE_RECORDING)
  {
    cap_tick = cap_tick + 1;
    if (cap_tick >= cap_divider)
    {
      cap_tick = 0;
      const uint16_t w = cap_write;
      const int16_t a = Scope_signal(cap_source[0]);
      scope_buf[0][w] = a;
      scope_buf[1][w] = Scope_signal(cap_source[1]);
      scope_buf[2][w] = Scope_signal(cap_source[2]);
      if (st == CAPTURE_ARMED)
      {
        bool fire = false;
        switch (cap_mode)
        {
        case TRIGGER_NOW:
          fire = true;
          break;
        case TRIGGER_ERROR:
          fire = cap_error_edge;
          break;
        case TRIGGER_RISING:
          fire = cap_filled > 0 && cap_prev_a < cap_level && a >= cap_level;
          break;
        case TRIGGER_FALLING:
          fire = cap_filled > 0 && cap_prev_a > cap_level && a <= cap_level;
          break;
        default:
          break;
        }
        if (fire)
        {
          const uint16_t pre = (cap_pretrigger < cap_filled) ? cap_pretrigger : cap_filled;
          cap_trigger_pos = w;
          cap_triggered = true;
          cap_after = SCOPE_LEN - pre;
          cap_state = CAPTURE_RECORDING;
        }
      }
      cap_prev_a = a;
      cap_write = (w + 1) & (SCOPE_LEN - 1);
      if (cap_filled < SCOPE_LEN)
        cap_filled = cap_filled + 1;
      if (cap_state == CAPTURE_RECORDING)
      {
        cap_after = cap_after - 1;
        if (cap_after == 0)
          cap_state = CAPTURE_DONE;
      }
    }
  }
  cogging_ff_last = 0; // the next loop sets it again if it applies the feedforward
}

/// @brief Start (mode 1-4) or stop (0) a capture. Stopping keeps what was recorded.
static void Capture_set_trigger(uint8_t mode)
{
  if (mode > TRIGGER_FALLING)
    mode = TRIGGER_STOP;
  const uint32_t irq = __get_PRIMASK();
  __disable_irq();
  cap_mode = mode;
  if (mode == TRIGGER_STOP)
  {
    if (cap_state == CAPTURE_ARMED || cap_state == CAPTURE_RECORDING)
      cap_state = (cap_filled > 0) ? CAPTURE_DONE : CAPTURE_IDLE;
  }
  else
  {
    cap_write = 0;
    cap_filled = 0;
    cap_after = 0;
    cap_triggered = false;
    cap_tick = cap_divider - 1; // first sample on the next loop
    cap_prev_a = 0;
    cap_prev_error = controller.Error; // an error already present does not fire the trigger
    cap_error_edge = false;
    cap_state = CAPTURE_ARMED;
  }
  __set_PRIMASK(irq);
}

/// @brief Start a histogram over x (clears it), or stop it (HIST_X_OFF, keeps the data).
static void Hist_set_x(uint8_t x)
{
  if (x > HIST_X_ENCODER)
    x = HIST_X_OFF;
  hist_x = HIST_X_OFF;
  COMPILER_BARRIER();
  if (x != HIST_X_OFF)
  {
    memset(hist_sum, 0, sizeof(hist_sum));
    memset(hist_count, 0, sizeof(hist_count));
    hist_samples = 0;
    COMPILER_BARRIER();
    hist_x = x;
  }
}

/// @brief Ring position of chronological sample i (0 = oldest).
static inline uint16_t Capture_pos(uint16_t i)
{
  const uint16_t start = (cap_filled < SCOPE_LEN) ? 0 : cap_write;
  return (start + i) & (SCOPE_LEN - 1);
}

static inline void Put16(uint8_t *d, int32_t v)
{
  d[0] = (uint8_t)((v >> 8) & 0xFF);
  d[1] = (uint8_t)(v & 0xFF);
}

static inline int16_t Get16(const uint8_t *d)
{
  return (int16_t)(((uint16_t)d[0] << 8) | d[1]);
}

static uint16_t Buffer_len(uint8_t kind)
{
  switch (kind)
  {
  case BUFFER_CAPTURE:
    return cap_filled;
  case BUFFER_HISTOGRAM:
    return SCOPE_HIST_BINS;
  case BUFFER_COGGING_HARMONICS:
    return COGGING_HARMONICS;
  case BUFFER_COGGING_TABLE:
    return COGGING_TABLE_LEN;
  case BUFFER_ENCODER_TABLE:
    return ENCAL_TABLE_LEN;
  default:
    return 0;
  }
}

/// @brief Fill one OUT_BUFFER_DATA frame: kind << 13 | index, then 6 bytes (constants.h).
static void Buffer_frame(uint8_t kind, uint16_t index, uint8_t *d)
{
  Put16(d, ((uint16_t)kind << 13) | (index & 0x1FFF));
  memset(d + 2, 0, 6);
  switch (kind)
  {
  case BUFFER_CAPTURE:
  {
    const uint16_t p = Capture_pos(index);
    Put16(d + 2, scope_buf[0][p]);
    Put16(d + 4, scope_buf[1][p]);
    Put16(d + 6, scope_buf[2][p]);
    break;
  }
  case BUFFER_HISTOGRAM:
  {
    const int32_t sum = hist_sum[index];
    Put16(d + 2, hist_count[index]);
    Put16(d + 4, sum >> 16);
    Put16(d + 6, sum);
    break;
  }
  case BUFFER_COGGING_HARMONICS:
    Put16(d + 2, cogging.harmonic[index]);
    Put16(d + 4, cogging.a[index]);
    Put16(d + 6, cogging.b[index]);
    break;
  case BUFFER_COGGING_TABLE:
    for (int k = 0; k < 3; k++)
      if (index + k < COGGING_TABLE_LEN)
        Put16(d + 2 + 2 * k, cogging.table[index + k]);
    break;
  case BUFFER_ENCODER_TABLE:
    for (int k = 0; k < 3; k++)
      if (index + k < ENCAL_TABLE_LEN)
        Put16(d + 2 + 2 * k, encoder.table[index + k]);
    break;
  default:
    break;
  }
}

/// @brief IN_BUFFER_READ: kind, first index (u16), count (u16). Replaces a read-out still in progress.
void Scope_read_request(const uint8_t *data)
{
  const uint8_t kind = data[0];
  const uint16_t first = ((uint16_t)data[1] << 8) | data[2];
  uint16_t count = ((uint16_t)data[3] << 8) | data[4];
  const uint16_t len = Buffer_len(kind);
  if (first >= len)
    count = 0;
  else if (count > len - first)
    count = len - first;
  out_kind = kind;
  out_next = first;
  out_end = first + count;
}

/// @brief Main loop: send the next read-out frames while a transmit mailbox is free (never waits).
void Scope_pump()
{
  for (int n = 0; n < 3 && out_next < out_end; n++)
  {
    if (!(CAN1->TSR & STM32_CAN_TSR_TME_ANY))
      return;
    CAN_msg_t msg;
    Buffer_frame(out_kind, out_next, msg.data);
    msg.len = 8;
    msg.type = DATA_FRAME;
    msg.format = STANDARD_FORMAT;
    msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_BUFFER_DATA, controller.Error);
    if (!CANSend(&msg))
      return;
    out_next += (out_kind == BUFFER_COGGING_TABLE || out_kind == BUFFER_ENCODER_TABLE) ? 3 : 1;
  }
}

/// @brief IN_BUFFER_WRITE: the anti-cogging harmonics and table, in RAM until IN_SAVE_CONFIG.
void Scope_write(const uint8_t *data)
{
  const uint16_t header = ((uint16_t)data[0] << 8) | data[1];
  const uint8_t kind = header >> 13;
  const uint16_t index = header & 0x1FFF;
  if (kind == BUFFER_COGGING_HARMONICS && index < COGGING_HARMONICS)
  {
    const uint16_t h = (uint16_t)Get16(data + 2);
    cogging.harmonic[index] = 0; // off while a and b change
    COMPILER_BARRIER();
    cogging.a[index] = Get16(data + 4);
    cogging.b[index] = Get16(data + 6);
    COMPILER_BARRIER();
    cogging.harmonic[index] = (h <= COGGING_MAX_HARMONIC) ? (uint8_t)h : 0;
  }
  else if (kind == BUFFER_COGGING_TABLE)
  {
    for (int k = 0; k < 3; k++)
      if (index + k < COGGING_TABLE_LEN)
        cogging.table[index + k] = Get16(data + 2 + 2 * k);
  }
}

bool Scope_param_get(uint8_t param, int32_t *value)
{
  switch (param)
  {
  case PARAM_CAPTURE_SOURCE_A:
  case PARAM_CAPTURE_SOURCE_B:
  case PARAM_CAPTURE_SOURCE_C:
    *value = cap_source[param - PARAM_CAPTURE_SOURCE_A];
    return true;
  case PARAM_CAPTURE_DIVIDER:
    *value = cap_divider;
    return true;
  case PARAM_CAPTURE_TRIGGER:
    *value = cap_mode;
    return true;
  case PARAM_CAPTURE_LEVEL:
    *value = cap_level;
    return true;
  case PARAM_CAPTURE_PRETRIGGER:
    *value = cap_pretrigger;
    return true;
  case PARAM_CAPTURE_STATE:
    *value = cap_state;
    return true;
  case PARAM_CAPTURE_COUNT:
    *value = cap_filled;
    return true;
  case PARAM_CAPTURE_TRIGGER_INDEX:
  {
    const uint16_t start = (cap_filled < SCOPE_LEN) ? 0 : cap_write;
    *value = cap_triggered ? (int32_t)((cap_trigger_pos - start) & (SCOPE_LEN - 1)) : -1;
    return true;
  }
  case PARAM_HIST_X:
    *value = hist_x;
    return true;
  case PARAM_HIST_Y:
    *value = hist_y;
    return true;
  case PARAM_HIST_SAMPLES:
    *value = (int32_t)hist_samples;
    return true;
  case PARAM_COGGING_MODE:
    *value = cogging.mode;
    return true;
  case PARAM_COGGING_SCALE:
    *value = cogging.scale;
    return true;
  case PARAM_CAPTURE_LENGTH:
    *value = SCOPE_LEN;
    return true;
  case PARAM_COGCAL:
    *value = Encal_chain_state(Cogcal_state());
    return true;
  case PARAM_COGCAL_SPEED:
    *value = cogcal_speed;
    return true;
  case PARAM_COGCAL_RIPPLE:
    *value = cogcal_ripple_x10;
    return true;
  default:
    return Encal_param_get(param, value); // 52-57: the encoder table
  }
}

bool Scope_param_set(uint8_t param, int32_t value)
{
  switch (param)
  {
  case PARAM_CAPTURE_SOURCE_A:
  case PARAM_CAPTURE_SOURCE_B:
  case PARAM_CAPTURE_SOURCE_C:
    cap_source[param - PARAM_CAPTURE_SOURCE_A] = (value >= 0 && value < SIGNAL_COUNT) ? (uint8_t)value : SIGNAL_NONE;
    return true;
  case PARAM_CAPTURE_DIVIDER:
    cap_divider = (value < 1) ? 1 : (value > 65535 ? 65535 : (uint16_t)value);
    return true;
  case PARAM_CAPTURE_TRIGGER:
    Capture_set_trigger((value >= 0 && value <= TRIGGER_FALLING) ? (uint8_t)value : TRIGGER_STOP);
    return true;
  case PARAM_CAPTURE_LEVEL:
    cap_level = Sat16(value);
    return true;
  case PARAM_CAPTURE_PRETRIGGER:
    cap_pretrigger = (value < 0) ? 0 : (value > SCOPE_LEN - 1 ? SCOPE_LEN - 1 : (uint16_t)value);
    return true;
  case PARAM_HIST_X:
    Hist_set_x((value >= 0 && value <= HIST_X_ENCODER) ? (uint8_t)value : HIST_X_OFF);
    return true;
  case PARAM_HIST_Y:
    hist_y = (value >= 0 && value < SIGNAL_COUNT) ? (uint8_t)value : SIGNAL_NONE;
    return true;
  case PARAM_COGGING_MODE:
    if (value == COGGING_MODE_HARMONICS || value == COGGING_MODE_TABLE)
    {
      if (!Cogging_has_set((uint8_t)value))
        return false; // nothing identified or loaded for this mode: it stays as it was (batch 18)
      cogging.mode = (uint8_t)value;
    }
    else
      cogging.mode = COGGING_MODE_OFF;
    return true;
  case PARAM_COGGING_SCALE:
    cogging.scale = (value < 0) ? 0 : (value > COGGING_SCALE_MAX ? COGGING_SCALE_MAX : (uint8_t)value);
    return true;
  case PARAM_COGCAL:
    if (value == 0)
      Cogcal_abort();
    else if (value == 1 || value == 2)
      Cogcal_start(value == 2);
    else if (value == 3)
      Encal_start(false, 2); // the encoder table, then the anti-cogging, then save both
    else if (value == 4 || value == 5)
      Cogcal_start(value == 5, true); // with the current-sense zero trim
    return true;
  case PARAM_COGCAL_SPEED:
    cogcal_speed = (value < 100) ? 100 : (value > 20000 ? 20000 : (uint16_t)value);
    return true;
  default:
    return Encal_param_set(param, value); // 52-57: the encoder table; otherwise unknown or read-only
  }
}

/// @brief Start a histogram of signal y over x (cogcal.cpp).
void Scope_hist_begin(uint8_t x, uint8_t y)
{
  hist_y = y;
  Hist_set_x(x);
}

/// @brief Stop the histogram; its data stays readable.
void Scope_hist_end()
{
  Hist_set_x(HIST_X_OFF);
}

/// @brief A bin's mean in units of 1/16 of the signal; false if the bin has no samples.
bool Scope_hist_mean16(uint16_t bin, int32_t *mean16)
{
  if (bin >= SCOPE_HIST_BINS || hist_count[bin] == 0)
    return false;
  *mean16 = (int32_t)(((int64_t)hist_sum[bin] * 16) / hist_count[bin]);
  return true;
}

/// @brief The capture buffer as scratch memory (SCOPE_CHANNELS * SCOPE_LEN / 2 int32): stops and clears any capture.
int32_t *Scope_scratch()
{
  const uint32_t irq = __get_PRIMASK();
  __disable_irq();
  cap_state = CAPTURE_IDLE;
  cap_mode = TRIGGER_STOP;
  cap_filled = 0;
  cap_write = 0;
  cap_triggered = false;
  __set_PRIMASK(irq);
  return (int32_t *)scope_buf;
}
