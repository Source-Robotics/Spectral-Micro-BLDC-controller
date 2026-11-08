/// @file scope.h
/// @brief Capture and histogram (2026-11): record signals inside the control loop, read them and the anti-cogging
/// data out over CAN (IN_BUFFER_READ / OUT_BUFFER_DATA), write the anti-cogging data (IN_BUFFER_WRITE).
/// Configured through parameters 24-38 (constants.h). The same in the STEPFOC and Spectral firmwares.
#ifndef SCOPE_H
#define SCOPE_H

#include <stdint.h>

#define SCOPE_LEN 1024       // samples per channel, a power of two
#define SCOPE_CHANNELS 3
#define SCOPE_HIST_BINS 256  // over one turn (electrical or mechanical): 64 angle counts per bin
#define SCOPE_HIST_SHIFT 6

void Scope_tick();                           // at the end of every control loop (IT_callback)
void Scope_pump();                           // main loop: sends pending OUT_BUFFER_DATA frames
void Scope_read_request(const uint8_t *data); // IN_BUFFER_READ, DLC 5
void Scope_write(const uint8_t *data);        // IN_BUFFER_WRITE, DLC 8
bool Scope_param_get(uint8_t param, int32_t *value);
bool Scope_param_set(uint8_t param, int32_t value);
void Scope_hist_begin(uint8_t x, uint8_t y);  // for cogcal.cpp
void Scope_hist_end();
bool Scope_hist_mean16(uint16_t bin, int32_t *mean16);
int32_t *Scope_scratch();

#endif
