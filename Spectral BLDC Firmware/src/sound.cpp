/// Sound through the motor (batch 22): see sound.h. Identical in both firmwares.
#include <Arduino.h>
#include <string.h>
#include "sound.h"
#include "common.h"
#include "constants.h"
#include "cogging.h"
#include "qfplib-m3.h"

struct SoundChannel
{
  uint32_t phase; // 2^32 per turn of the sine
  uint32_t inc;   // per loop
  int32_t amp_mv;
  uint32_t left;  // loops left; 0xFFFFFFFF = until stopped
  uint8_t on;
};
struct SoundEvent
{
  uint16_t freq;   // Hz; 0 = silence this channel
  uint16_t amp_mv;
  uint16_t ms;     // then wait this long before the next event (0: the next one starts with it)
  uint8_t channel;
};

static volatile SoundChannel ch[SOUND_CHANNELS];
static SoundEvent queue[SOUND_QUEUE_LEN];
static volatile uint16_t q_head = 0; // the main loop appends here
static volatile uint16_t q_tail = 0; // the interrupt takes from here
static volatile uint8_t q_running = 0;
static volatile uint32_t q_wait = 0; // loops until the next event

static int8_t ring[SOUND_RING_LEN];
static volatile uint16_t r_head = 0, r_tail = 0;
static volatile uint8_t r_playing = 0, r_underrun = 0;
static volatile uint32_t r_inc = 0, r_pos = 0; // samples per loop, 16.16
static volatile int32_t r_vol_mv = 0;
static volatile int32_t r_cur = 0;
static volatile uint8_t any = 0; // something may sound (cheap check for the interrupt)

static uint16_t be16(const uint8_t *p)
{
  return (uint16_t)((p[0] << 8) | p[1]);
}

static uint32_t inc_for(uint32_t f)
{
  return (uint32_t)(((uint64_t)f << 32) / LOOP_FREQ);
}

static uint32_t loops_for(uint32_t ms)
{
  return (uint32_t)(((uint64_t)ms * LOOP_FREQ + 500u) / 1000u);
}

static int32_t limit_mv()
{
  return controller.sound_limit_mv;
}

/// The voltage for a current at this pitch: the coil's impedance, sqrt(R^2 + (2 pi f L)^2) (1 ohm if unknown).
static uint16_t mv_for(uint32_t freq, uint32_t ma)
{
  const float r = controller.Resistance;
  const float xl = PI2 * (float)freq * controller.Inductance;
  float z = qfp_fsqrt(r * r + xl * xl);
  if (!(z > 0.01f))
    z = 1.0f;
  float mv = (float)ma * z;
  const float lim = (float)limit_mv();
  if (mv > lim)
    mv = lim;
  return (uint16_t)(mv + 0.5f);
}

static bool freq_ok(uint32_t f)
{
  return f >= 20 && f < LOOP_FREQ / 2;
}

bool Sound_active()
{
  return any != 0;
}

void Sound_stop()
{
  const uint32_t irq = __get_PRIMASK();
  __disable_irq();
  for (int k = 0; k < SOUND_CHANNELS; k++)
    ch[k].on = 0;
  q_running = 0;
  q_head = q_tail = 0;
  q_wait = 0;
  r_playing = 0;
  r_head = r_tail = 0;
  r_cur = 0;
  any = 0;
  __set_PRIMASK(irq);
}

bool Sound_tone(uint8_t channel, uint16_t freq_hz, uint16_t amp_ma, uint16_t ms)
{
  if (channel >= SOUND_CHANNELS || limit_mv() <= 0)
    return false;
  const uint32_t irq = __get_PRIMASK();
  __disable_irq();
  volatile SoundChannel &c = ch[channel];
  if (freq_hz == 0 || amp_ma == 0 || !freq_ok(freq_hz))
    c.on = 0;
  else
  {
    c.inc = inc_for(freq_hz);
    c.amp_mv = mv_for(freq_hz, amp_ma);
    c.left = ms ? loops_for(ms) : 0xFFFFFFFFu;
    c.on = 1;
    any = 1;
  }
  __set_PRIMASK(irq);
  return true;
}

void Sound_command(const uint8_t *data, uint8_t len)
{
  if (len < 8)
    return;
  const uint8_t op = data[0], channel = data[1];
  const uint16_t a = be16(data + 2), b = be16(data + 4), c = be16(data + 6);
  switch (op)
  {
  case 0: // stop everything
    Sound_stop();
    break;
  case 1: // a tone now: channel, freq Hz, current mA, duration ms (0: until stopped)
    Sound_tone(channel, a, b, c);
    break;
  case 2: // queue an event: channel, freq Hz (0: silence), current mA, then wait ms
  {
    if (channel >= SOUND_CHANNELS || limit_mv() <= 0)
      break;
    const uint16_t next = (uint16_t)((q_head + 1) % SOUND_QUEUE_LEN);
    if (next == q_tail)
      break; // full: dropped (parameter 80 says how many are queued)
    SoundEvent &e = queue[q_head];
    e.freq = freq_ok(a) ? a : 0;
    e.amp_mv = e.freq ? mv_for(a, b) : 0;
    e.ms = c;
    e.channel = channel;
    __asm volatile("" ::: "memory"); // the event is written before the interrupt may take it
    q_head = next;
    break;
  }
  case 3: // play the queue
    if (limit_mv() <= 0)
      break;
    q_wait = 0;
    q_running = 1;
    any = 1;
    break;
  case 4: // samples: rate Hz, full-scale voltage mV; send samples first, then this
  {
    if (limit_mv() <= 0 || a == 0)
      break;
    const uint32_t rate = (a > LOOP_FREQ) ? LOOP_FREQ : a;
    int32_t vol = b;
    if (vol > limit_mv())
      vol = limit_mv();
    const uint32_t irq = __get_PRIMASK();
    __disable_irq();
    r_inc = (rate << 16) / LOOP_FREQ;
    r_pos = 0;
    r_vol_mv = vol;
    r_underrun = 0;
    r_playing = 1;
    any = 1;
    __set_PRIMASK(irq);
    break;
  }
  case 5: // samples off, buffer cleared
  {
    const uint32_t irq = __get_PRIMASK();
    __disable_irq();
    r_playing = 0;
    r_head = r_tail = 0;
    r_cur = 0;
    __set_PRIMASK(irq);
    break;
  }
  case 6: // clear the queue (tones that sound keep sounding)
  {
    const uint32_t irq = __get_PRIMASK();
    __disable_irq();
    q_running = 0;
    q_head = q_tail = 0;
    q_wait = 0;
    __set_PRIMASK(irq);
    break;
  }
  default:
    break;
  }
}

void Sound_samples(const uint8_t *data, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++)
  {
    const uint16_t next = (uint16_t)((r_head + 1) % SOUND_RING_LEN);
    if (next == r_tail)
      return; // full: the rest is dropped
    ring[r_head] = (int8_t)data[i];
    __asm volatile("" ::: "memory");
    r_head = next;
  }
}

int32_t Sound_tick()
{
  if (!any)
    return 0;
  // the queue: take events until one says wait
  if (q_running)
  {
    while (q_wait == 0)
    {
      if (q_tail == q_head)
      {
        q_running = 0;
        for (int k = 0; k < SOUND_CHANNELS; k++)
          ch[k].on = 0;
        break;
      }
      const SoundEvent &e = queue[q_tail];
      volatile SoundChannel &c = ch[e.channel];
      if (e.freq == 0 || e.amp_mv == 0)
        c.on = 0;
      else
      {
        c.inc = inc_for(e.freq);
        c.amp_mv = e.amp_mv;
        c.left = 0xFFFFFFFFu;
        c.on = 1;
      }
      q_wait = loops_for(e.ms);
      q_tail = (uint16_t)((q_tail + 1) % SOUND_QUEUE_LEN);
    }
    if (q_wait)
      q_wait--;
  }
  // the tones
  int32_t v = 0;
  uint8_t on = 0;
  for (int k = 0; k < SOUND_CHANNELS; k++)
  {
    volatile SoundChannel &c = ch[k];
    if (!c.on)
      continue;
    if (c.left != 0xFFFFFFFFu)
    {
      if (c.left == 0)
      {
        c.on = 0;
        continue;
      }
      c.left = c.left - 1;
    }
    c.phase = c.phase + c.inc;
    v += (c.amp_mv * Cogging_sin_q15(c.phase >> 18)) >> 15;
    on = 1;
  }
  // the samples
  if (r_playing)
  {
    r_pos = r_pos + r_inc;
    while (r_pos >= 65536u)
    {
      r_pos = r_pos - 65536u;
      if (r_tail == r_head)
      {
        r_playing = 0; // ran dry
        r_underrun = 1;
        r_cur = 0;
        break;
      }
      r_cur = ring[r_tail];
      r_tail = (uint16_t)((r_tail + 1) % SOUND_RING_LEN);
    }
    v += (r_vol_mv * r_cur) / 127;
  }
  any = (on || q_running || r_playing) ? 1 : 0;
  const int32_t lim = limit_mv();
  if (v > lim)
    v = lim;
  else if (v < -lim)
    v = -lim;
  return v;
}

int32_t Sound_state()
{
  const uint32_t queued = (uint32_t)((q_head + SOUND_QUEUE_LEN - q_tail) % SOUND_QUEUE_LEN);
  const uint32_t buffered = (uint32_t)((r_head + SOUND_RING_LEN - r_tail) % SOUND_RING_LEN);
  uint32_t tones = 0;
  for (int k = 0; k < SOUND_CHANNELS; k++)
    if (ch[k].on)
      tones = 1;
  return (int32_t)(queued | (buffered << 8) | (tones << 24) | ((uint32_t)q_running << 25) |
                   ((uint32_t)r_playing << 26) | ((uint32_t)r_underrun << 27));
}
