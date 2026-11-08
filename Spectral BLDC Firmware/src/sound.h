#pragma once
/// Sound through the motor (2026-11, batch 22): tones, a queue of notes and streamed samples, added to the output
/// voltage on the q axis (a small torque ripple; parameter 82 = 0 picks the quieter d axis) while the drive holds or moves (closed-loop
/// modes 1-4). Identical in both firmwares.
///
/// Up to SOUND_CHANNELS tones at once (a chord on one motor); each from a phase accumulator and the integer sine
/// table, loudness given as a current (mA) and turned into a voltage with the coil's impedance at that pitch
/// (sqrt(R^2 + (2 pi f L)^2)), so a note is as loud low as high. The queue is a list of events (set a channel's
/// pitch and loudness, then wait), played by the control interrupt with loop-exact timing, so a host only loads it
/// and starts it; several drives started one after another are within a millisecond of each other. Samples (int8)
/// are played from a ring buffer at a rate the host sets (up to the loop rate). The total is limited to parameter
/// 81 (mV; 0 refuses sound). Anything other than a closed-loop mode, or an error, stops everything.
#include <stdint.h>

#define SOUND_CHANNELS 4        // tones at once
#define SOUND_QUEUE_LEN 32      // queued events
#define SOUND_RING_LEN 1024     // buffered samples

void Sound_command(const uint8_t *data, uint8_t len); // CAN IN_SOUND (44)
void Sound_samples(const uint8_t *data, uint8_t len); // CAN IN_SOUND_SAMPLES (45)
bool Sound_tone(uint8_t channel, uint16_t freq_hz, uint16_t amp_ma, uint16_t ms); // a tone now (also #Tone)
void Sound_stop();              // everything off, queue and samples cleared
bool Sound_active();            // something is or may be sounding
int32_t Sound_tick();           // control interrupt, once per loop in a closed-loop mode: the sound voltage, mV
int32_t Sound_state();          // parameter 80
