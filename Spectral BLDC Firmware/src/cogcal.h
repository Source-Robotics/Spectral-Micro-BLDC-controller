/// @file cogcal.h
/// @brief Anti-cogging identification on the drive (2026-11): the drive turns itself one motor turn forward and one
/// back in velocity mode at the cogcal speed, records the histogram of the loop's own current by electrical angle
/// (scope.cpp), fits the 8 largest harmonics of 1-32 and the 256-value table (cogging.h) from forward and backward
/// averaged, switches the harmonics on and, if asked, saves. Started by #Cogcal (UART) or parameter 43 (CAN); a
/// command from a host or an error aborts it. It ends holding its position. The same in both firmwares.
#ifndef COGCAL_H
#define COGCAL_H

#include <stdint.h>

void Cogcal_start(bool save, bool trim = false); // trim: correct the current-sense zeros from the 1st harmonic, identify again
void Cogcal_abort();
void Cogcal_run();               // main loop, every pass
uint8_t Cogcal_state();          // COGCAL_*
void Cogcal_mark_stale();        // the set no longer fits the encoder (encal.cpp): COGCAL_STALE (batch 18)
extern volatile uint16_t cogcal_speed;      // ticks/s, parameter 44
extern volatile int32_t cogcal_ripple_x10;  // the identified cogging current, rms, 0.1 mA, parameter 45
extern volatile int32_t cogcal_h1_before_x10, cogcal_h1_after_x10; // the 1st harmonic around a trim, 0.1 mA

#endif
