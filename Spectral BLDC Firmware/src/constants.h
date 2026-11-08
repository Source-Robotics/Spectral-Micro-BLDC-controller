/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    constants.h
 * @brief   This file provides code where we declare constants
 * @author Petar Crnjak
 ******************************************************************************
 * @attention
 *
 * Copyright (c) Source robotics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONSTANTS_H
#define CONSTANTS_H

// Firmware/release version. BUMP THIS on every release so #Info reports the actual
// firmware loaded (not a stale value from EEPROM). Should match the GitHub release tag (Vxxx).
#define FIRMWARE_VERSION 107

#define MAX_DRIVE_CURRENT 2800 // 2500 mA

#define MT6816_NO_MAGNET_BIT 0x0002

#define MAX_DUTY 0.95f

#define MIN_DUTY 0.05f

#define OVERMODULATION 1.154 // 1 no overmodulation, 1.154F SPWM overmodulation

#define ADC_RESOLUTION_ 12

#define MAX_TEMPERATURE 80

#ifdef PI
#undef PI // Arduino core's wiring_constants.h also defines PI; ours is used project-wide instead
#endif
#define PI 3.14159265359f

#define PI2 6.283185307f

#define SQRT3DEV2 0.8660254038f

#define CPR 16384

#define RAD_CONST 0.000383495187f

#define CPR2 (16384 / 2)

#define MIN_VOLTAGE

#define PWM_FREQ 25000

#define LOOP_FREQ 5000 

#define LOOP_TIME 0.0002 

#define CLOCK_PRESCALE 16

#define SERIAL_SPEED 256000 // 256000 works, Tested with 1Mbit and works

#define SENSE_RESISTOR 0.025f

#define Rdson 0.24f

#define ADC_MIDPOINT 1.65f

#define ADC_MIDPOINT_BIT 2048

#define CURRENT_AMP_GAIN 20

#define R1_voltage 100000

#define R2_voltage 870

#define PWM_MAX 8191

#define PWM_MIN 0

/*
EEPROM CONSTANTS
*/

///// EEPROM variable address list
#define SERIAL_NUMBER_EEPROM 32
#define HARDWARE_VERSION_EEPROM 36
#define BATCH_DATA_EEPROM 40
#define CAN_ID_EEPROM 44
#define SOFTWARE_VERSION_EEPROM 48
#define LED_ON_OFF_EEPROM 52
#define THERMISTOR_ON_OFF_EEPROM 56
#define POLE_PAIR 60
#define DIR_EEPROM 64
#define RESISTANCE_EEPROM 68
#define TOTAL_RESISTANCE_EEPROM 72
#define INDUCTANCE_EEPROM 76
#define KT_EEPROM 80
#define KV_EEPROM 84
#define FLUX_LINKAGE_EEPROM 92
#define KPP_EEPROM 96
#define KPV_EEPROM 100
#define KIV_EEPROM 104
#define VELOCITY_LIMIT_EEPROM 108
#define KIIQ_EEPROM 112
#define KPIQ_EEPROM 116
#define IQ_CURRENT_LIMIT_EEPROM 120

#define KIID_EEPROM 124
#define KPID_EEPROM 128
#define ID_CURRENT_LIMIT_EEPROM 132

#define KP_EEPROM 136
#define KD_EEPROM 140
#define CALIBRATED_EEPROM 144
#define PHASE_ORDER_EEPROM 148
#define WATCHDOG_TIME_EEPROM 152
#define WATCHDOG_ACTION_EEPROM 156
#define HEARTBEAT_RATE_EEPROM 160
#define I_AM_GRIPPER_EEPROM 164
#define RESET_INTEGRAL_EEPROM 168
#define TEMPERATURE_ERROR 172
#define VOLTAGE_ERROR 176
#define VOLTAGE_LIMIT 180
#define THETA_OFFSET 184
#define COMMUTATION_DIR_EEPROM 188 // stored 2 = reversed wiring, else forward (+1). Next free slot after THETA_OFFSET.

// Two independent CRCs instead of one monolithic config CRC: a future firmware update that
// adds a new SETTINGS field can only ever invalidate SETTINGS_CRC_EEPROM, never
// CALIBRATION_CRC_EEPROM -- so it can't force a recalibration on hardware that may be hard
// to physically reach. See EEPROM.cpp's read_config()/Compute_*_crc_from_eeprom().
#define SETTINGS_CRC_EEPROM 196
// No gap to 208 here (unlike the sibling stepper firmware): that gap there exists only
// because ITS deployed boards previously held an experimental feature at 196/200/204. This
// BLDC firmware never had that feature, so there's nothing to leave unused -- the layout is
// packed tight instead.
#define CALIBRATION_CRC_EEPROM 200

// --- RESERVED, NOT CURRENTLY USED ---
// Pre-reserved headroom in the calibration block for future calibration-adjacent features
// (e.g. magnet alignment, sensorless-control observer parameters). Nothing reads or writes
// these for any real purpose yet -- they are always 0. They ARE included in
// CALIB_FLOAT_ADDRS/CALIB_INT_ADDRS (EEPROM.cpp) and therefore in the calibration CRC, so
// that a future feature can start USING one of these addresses without changing the
// calibration block's address list -- growing that list is what would invalidate every
// already-sealed board's CRC (see the comment above Compute_calibration_crc_from_eeprom()).
// A dedicated flags int is included so a future feature can mark "configured" independent
// of whatever value(s) it stores (a value of 0 is otherwise ambiguous with "never set").
//
// IMPORTANT: this only works because these slots are being added now, before any board has
// ever sealed a calibration CRC that includes them (this firmware release is what introduces
// the CRC in the first place). If more headroom is needed AFTER boards are deployed with a
// CRC sealed over this exact list, do NOT grow this list again -- give the new feature its
// own independent block/CRC instead (the same pattern used to split Calibration from
// Settings). A large/variable-size feature (e.g. an anticogging lookup table) should get its
// own dedicated block regardless, not use these scalar slots.
#define CALIB_RESERVED_F1_EEPROM 204     // unused float, reserved for a future feature
#define CALIB_RESERVED_F2_EEPROM 208     // unused float, reserved for a future feature
#define CALIB_RESERVED_F3_EEPROM 212     // unused float, reserved for a future feature
#define CALIB_RESERVED_F4_EEPROM 216     // unused float, reserved for a future feature
#define CALIB_RESERVED_I1_EEPROM 220     // unused int, reserved for a future feature
#define CALIB_RESERVED_I2_EEPROM 224     // unused int, reserved for a future feature
#define CALIB_FEATURE_FLAGS_EEPROM 228   // unused bitmask int, reserved for future "is configured" flags

// --- Settings block 2 (2026-11) ---
// Settings block 1 (SETTINGS_INT_ADDRS/SETTINGS_FLOAT_ADDRS in EEPROM.cpp) is FROZEN: adding an address to it
// changes its CRC on every deployed board and resets their settings at the next boot. New settings go into
// this block, and later into its reserved slots; when those run out, add a block 3. Never edit the address
// list of a block that has shipped.
// It starts at 256, the 24C16's second 256-byte bank (I2C address 0x51), at the same addresses as in the
// STEPFOC firmware; 232-255 stay free. read_config() checks that the bank answers before using it.
// No legacy heuristic: these fields never existed on an older board, so defaults are always right for an
// unsealed block.
#define SETTINGS2_CRC_EEPROM 256
#define TOOL_ID_EEPROM 260               // int: 0 none, 1 SSG-48, 11-33 MSG (tens = rail, units = stepper), 200-254 custom
#define WD_FALLBACK_TIME_EEPROM 264      // int ms: how long watchdog hold/descend may run
#define WD_FALLBACK_CURRENT_EEPROM 268   // int mA: the current descend applies
#define WD_FALLBACK_SPEED_EEPROM 272     // int ticks/s: the speed cap of descend
#define WD_AFTER_ACTION_EEPROM 276       // int: after hold/descend, 0 legacy, 2 brake, 3 coast
#define BOARD_PRODUCT_EEPROM 280        // int: product lock stamp, the product that calibrated this board (1, 2; 0 none)
#define SETTINGS2_RESERVED_I2_EEPROM 284 // unused int, reserved
#define SETTINGS2_RESERVED_I3_EEPROM 288 // unused int, reserved
#define SETTINGS2_RESERVED_I4_EEPROM 292 // unused int, reserved
#define SETTINGS2_RESERVED_F1_EEPROM 296 // unused float, reserved
#define SETTINGS2_RESERVED_F2_EEPROM 300 // unused float, reserved
#define SETTINGS2_RESERVED_F3_EEPROM 304 // unused float, reserved
#define SETTINGS2_RESERVED_F4_EEPROM 308 // unused float, reserved
#define SETTINGS2_FEATURE_FLAGS_EEPROM 312 // unused bitmask, reserved for "is configured" flags

// Anti-cogging block 3 (2026-11): its own CRC, so neither it nor the blocks above can invalidate each other.
// 320: int holding a CRC-16 over the block; 324-879: the CoggingBlock struct (cogging.h, 556 bytes). It reaches the
// fourth 256-byte bank (I2C 0x53); without it the block is not used and anti-cogging stays off at start-up.
#define COGGING_CRC_EEPROM 320
#define COGGING_BLOCK_EEPROM 324
// The encoder table block (encal.h, batch 9): CRC at 880, the block at 884-1143, into the fifth bank (I2C 0x54).
#define ENCODER_CRC_EEPROM 880
#define ENCODER_BLOCK_EEPROM 884
#define ENCODER_BLOCK_VERSION 2 // 2: with the current-sense zero trims

#define WD_FALLBACK_TIME_MAX_MS 60000    // hold/descend never run longer than this

// Undervoltage is not checked for this long after start-up: the supply may still be ramping (overvoltage always is)
#define VBUS_STARTUP_GRACE_MS 500

// Current-sense zero, measured at start-up with the bridge off (motor_control.cpp Sense_zero_calibrate)
#define SENSE_ZERO_SAMPLES 256           // readings averaged per channel
#define SENSE_ZERO_SW_SETTLE 64       // control loops at 50 % duty before the switching zero is averaged
#define SENSE_ZERO_SW_SAMPLES 512     // control loops averaged for the switching zero
#define SENSE_ZERO_SW_MAX_SHIFT 50    // counts the switching zero may differ from the bridge-off zero
#define SENSE_ZERO_SW_NOT_RUN 0
#define SENSE_ZERO_SW_RUNNING 1
#define SENSE_ZERO_SW_IN_USE 2
#define SENSE_ZERO_SW_REJECTED 3      // implausible: the bridge-off zero stays
#define SENSE_ZERO_SW_INTERRUPTED 4   // a command or an error came first: the bridge-off zero stays
#define SENSE_ZERO_SW_MEASURED 5      // measured and plausible, not in use (parameter 46 switches it in)
#define SENSE_ZERO_MAX_DEV 200           // counts from 2048 beyond which the zero is not trusted (2048 is kept)

// Which firmware this is, reported in OUT_IN_IDENTITY byte 0: 1 = STEPFOC, 2 = Spectral Micro
#define PRODUCT_ID 2
/*
CAN BUS CONSTANTS
*/

// OUTPUT CAN command IDs (Commands that spectral driver can send)
#define OUT_HEARTBEAT 9
#define OUT_DATA_PACK_1 3
#define OUT_DATA_PACK_2 5
#define OUT_DATA_PACK_3 7

// Commands that host and driver can send. Host only with RTR frame
// Commands that have same command ID on PC and driver side by using REMOTE FRAME
// When beign sent by host they need to be sent as REMOTE FRAME
// BLDC driver then responds with data with SAME ID as the REMOTE FRAME
#define OUT_IN_TEMPERATURE_CAN 23
#define OUT_IN_VOLTAGE_CAN 24
#define OUT_IN_DEVICE_INFO_CAN 25
#define OUT_IN_STATE_OF_ERRORS_CAN 26
#define OUT_IN_IQ 27
#define OUT_IN_ENCODER 28
#define OUT_IN_PING 10
#define OUT_IN_KT 33

// INPUT CAN command IDS (Commands that spectral driver can receive)
// To these commands spectral responds nothing
#define IN_CAN_ID 11
#define IN_ESTOP 0
#define IN_IDLE 12
#define IN_SAVE_CONFIG 13
#define IN_RESET 14
#define IN_CLEAR_ERROR 1
#define IN_WATCHDOG_TIMEOUT 15
#define HEARTBEAT_SETUP 30
#define IN_CYCLIC 29

#define IN_KP_KD 16
#define IN_KIIQ_KPIQ 17
#define IN_KPV_KIV 18
#define IN_KPP 19
#define IN_LIMITS 20
#define IN_IQ 21
#define IN_KT 22
#define IN_VLIM 34 // PID voltage limit (PID.Voltage_limit), 4-byte int payload in mV

// Generic mask+value settings command: data[0] = mask, data[1] = value, one bit per setting.
// Only bits set in the mask are applied, so a sender only needs to know about the settings
// bits it actually cares about. To add a new setting: pick the next free bit, add one
// SETTINGS_BIT_* define, add one "if (mask & ...)" block in the IN_SETTINGS handler.
#define IN_SETTINGS 35
#define SETTINGS_BIT_BRAKE_COAST (1 << 0) // 1 = brake, 0 = coast; idle-mode phase behavior (controller.brake_coast)

// Generic parameters (2026-11). A new setting gets a parameter number (PARAM_* below) instead of a command id,
// so settings no longer use up the 64 command ids.
// IN_PARAM_SET: data frame, DLC 5: parameter number (u8), value (i32, big-endian). No answer. Unknown numbers
//   are ignored; values outside a parameter's range are clamped (the tool id is only taken if 0-255).
//   Saved by IN_SAVE_CONFIG like every other setting.
// IN_PARAM_GET: data frame, DLC 1: parameter number. The drive answers on OUT_PARAM_VALUE.
// OUT_PARAM_VALUE: DLC 5: parameter number, the value in force (i32, big-endian); DLC 1: unknown parameter.
// The answer has an id of its own: a host and a drive must never send DATA frames with the same id, because two
// such frames sent at the same moment collide. (A remote-frame question and its answer may share an id, which
// is how the OUT_IN_* commands work.)
#define IN_PARAM_SET 36
#define IN_PARAM_GET 37
#define OUT_PARAM_VALUE 38
// Identity, remote frame only. Reply DLC 8: product (1 STEPFOC, 2 Spectral), tool id, firmware version (u16),
// hardware version, pole pairs, calibrated (0/1), flags (bit 0 = I_AM_GRIPPER, bit 1 = brake on idle,
// bit 2 = commutation reversed, bit 3 = settings block 2 available, bit 4 = wrong firmware for this board
// (product lock), bit 5 = current-sense zero measured, bit 6 = anti-cogging block 3 saved and loaded,
// bit 7 = current-sense zero measured with the bridge switching (in use);
// bit 0 is the least significant).
#define OUT_IN_IDENTITY 39
// Telemetry, remote frame only. Reply DLC 8, one snapshot: temperature degC (i16), Vbus mV (i16), error byte 1,
// error byte 2 (the OUT_IN_STATE_OF_ERRORS_CAN layout), Iq mA (i16), all big-endian. Replaces four requests.
#define OUT_IN_TELEMETRY 40
// Buffers (2026-11): the capture, the histogram and the anti-cogging data, read and written in 8-byte frames.
// IN_BUFFER_READ: data frame, DLC 5: kind (u8), first index (u16), count (u16), big-endian. The drive answers with
//   one OUT_BUFFER_DATA frame per index (the table: one per 3 entries), sent from the main loop as fast as the bus
//   takes them. A new request replaces one still in progress; count 0 cancels. Indexes past the end are not sent.
// OUT_BUFFER_DATA: DLC 8: header (u16) = kind << 13 | index, then 6 bytes by kind:
//   0 capture:   channel A, B, C (i16) of sample `index`, oldest first
//   1 histogram: count (u16), sum (i32) of bin `index`; the mean is sum / count
//   2 harmonics: harmonic (u16, 0 = slot unused), a (i16 mA, cosine), b (i16 mA, sine) of slot `index`
//   3 table:     entries index, index+1, index+2 (i16 mA; past the end 0)
// IN_BUFFER_WRITE: data frame, DLC 8, the OUT_BUFFER_DATA layout; kinds 2 and 3 only. In RAM until IN_SAVE_CONFIG.
#define IN_BUFFER_READ 41
#define OUT_BUFFER_DATA 42
#define IN_BUFFER_WRITE 43
#define BUFFER_CAPTURE 0
#define BUFFER_HISTOGRAM 1
#define BUFFER_COGGING_HARMONICS 2
#define BUFFER_COGGING_TABLE 3
#define BUFFER_ENCODER_TABLE 4      // the encoder table: 128 entries (1/16 count), 3 per frame; read only
// Sound through the motor (batch 22, sound.h). Data frames, big-endian, no answer:
// IN_SOUND, DLC 8: op (u8), channel (u8), a (u16), b (u16), c (u16):
//   op 0 stop everything; 1 a tone now (channel 0-3, a = Hz, b = mA, c = ms, 0 = until stopped);
//   2 queue an event (channel, a = Hz or 0 for silence, b = mA, then wait c ms; 0 = the next one starts with it);
//   3 play the queue; 4 play samples (a = sample rate Hz, b = full-scale mV; send samples first); 5 samples off;
//   6 clear the queue. Sound plays only while the drive holds or moves (modes 1-4); see parameters 80-82.
// IN_SOUND_SAMPLES, DLC 1-8: that many samples (i8), appended to the 1024-sample buffer.
#define IN_SOUND 44
#define IN_SOUND_SAMPLES 45
// Signals the capture and the histogram record (PARAM_CAPTURE_SOURCE_*, PARAM_HIST_Y), int16 each.
#define SIGNAL_NONE 0
#define SIGNAL_IQ 1                 // measured Iq, mA
#define SIGNAL_ID 2                 // measured Id, mA
#define SIGNAL_IQ_SETPOINT 3        // Iq setpoint, mA (with the anti-cogging)
#define SIGNAL_VELOCITY 4           // filtered velocity, ticks/s (saturates at +-32767)
#define SIGNAL_VELOCITY_16 5        // filtered velocity / 16, ticks/s
#define SIGNAL_POSITION 6           // position, ticks, low 16 bits (wraps; unwrap on the host)
#define SIGNAL_POSITION_ERROR 7     // position setpoint - position, ticks
#define SIGNAL_ELECTRICAL_ANGLE 8   // (encoder * pole pairs) mod 16384, the anti-cogging angle (no theta offset)
#define SIGNAL_ENCODER 9            // encoder, 0-16383 per motor turn
#define SIGNAL_VBUS 10              // Vbus, 10 mV
#define SIGNAL_SENSE1 11            // current sense 1, ADC counts after the zero
#define SIGNAL_SENSE2 12            // current sense 2, ADC counts after the zero
#define SIGNAL_LOOP_TIME 13         // control interrupt time of the previous loop, us
#define SIGNAL_COGGING_FF 14        // anti-cogging feedforward, mA
#define SIGNAL_IQ_SETPOINT_LOOP 15  // Iq setpoint without the anti-cogging: what the loop itself asks, mA
#define SIGNAL_UQ 16                // Uq setpoint, mV
#define SIGNAL_UD 17                // the d voltage the output transform received, mV (batch 12)
#define SIGNAL_UQ_APPLIED 18        // the q voltage the output transform received, mV
#define SIGNAL_COUNT 19
// Capture trigger (PARAM_CAPTURE_TRIGGER) and state (PARAM_CAPTURE_STATE)
#define TRIGGER_STOP 0              // stop; what was recorded stays readable
#define TRIGGER_NOW 1               // record SCOPE_LEN samples from now
#define TRIGGER_ERROR 2             // record until the general error rises (a watchdog trip too), keep pre-trigger
#define TRIGGER_RISING 3            // channel A rises through the level
#define TRIGGER_FALLING 4           // channel A falls through the level
#define CAPTURE_IDLE 0
#define CAPTURE_ARMED 1             // recording into the ring, waiting for the trigger
#define CAPTURE_RECORDING 2         // triggered, filling the rest
#define CAPTURE_DONE 3
// Histogram x (PARAM_HIST_X)
#define HIST_X_OFF 0                // stopped; the data stays readable
#define HIST_X_ELECTRICAL 1         // the electrical angle (SIGNAL_ELECTRICAL_ANGLE), 256 bins
#define HIST_X_ENCODER 2            // the encoder, one motor turn, 256 bins

// Parameter numbers for IN_PARAM_SET / IN_PARAM_GET. Permanent once released: never renumber or reuse one.
#define PARAM_TOOL_ID 1             // 0 none, 1 SSG-48, 11-33 MSG (tens = rail, units = stepper), 200-254 custom
#define PARAM_WD_FALLBACK_TIME 2    // ms that watchdog hold/descend may run, 0-60000
#define PARAM_WD_FALLBACK_CURRENT 3 // mA that descend applies, signed
#define PARAM_WD_FALLBACK_SPEED 4   // ticks/s, the speed cap of descend in either direction, 0-65535
#define PARAM_WD_AFTER_ACTION 5     // after hold/descend: 0 legacy, 2 brake, 3 coast
#define PARAM_WD_TIME 6             // watchdog time ms, 0 = off (also set by IN_WATCHDOG_TIMEOUT)
#define PARAM_WD_ACTION 7           // watchdog action 0-4 (also set by IN_WATCHDOG_TIMEOUT data[4])
// Read-only from here: the values in force, readable over CAN at last (set them with their own commands).
// F = the 4 value bytes are an IEEE-754 float (the bit pattern, big-endian), as the gain commands send them.
#define PARAM_KPP 8                 // F position gain (IN_KPP)
#define PARAM_KPV 9                 // F velocity P gain (IN_KPV_KIV)
#define PARAM_KIV 10                // F velocity I gain (IN_KPV_KIV)
#define PARAM_KPIQ 11               // F current P gain (IN_KIIQ_KPIQ)
#define PARAM_KIIQ 12               // F current I gain (IN_KIIQ_KPIQ)
#define PARAM_KP 13                 // F PD/impedance stiffness (IN_KP_KD)
#define PARAM_KD 14                 // F PD/impedance damping (IN_KP_KD)
#define PARAM_VELOCITY_LIMIT 15     // F velocity limit, ticks/s (IN_LIMITS)
#define PARAM_CURRENT_LIMIT 16      // F current limit, mA (IN_LIMITS)
#define PARAM_VOLTAGE_LIMIT 17      // voltage limit, mV, 0 = Vbus (IN_VLIM)
#define PARAM_BRAKE_COAST 18        // idle behaviour, 1 brake, 0 coast (IN_SETTINGS bit 0)
#define PARAM_HEARTBEAT_MS 19       // heartbeat period ms, 0 = off (HEARTBEAT_SETUP)
#define PARAM_KT 20                 // F torque constant (IN_KT)
#define PARAM_SENSE1_OFFSET 21      // current-sense zero of channel 1, ADC counts from 2048 (1 count = 1.6 mA);
                                    // writable, in RAM until the next start measures it again (batch 10)
#define PARAM_SENSE2_OFFSET 22      // current-sense zero of channel 2, ADC counts from 2048
#define PARAM_BOARD_PRODUCT 23      // product lock stamp: 1 STEPFOC, 2 Spectral, 0 none
// Capture, histogram and anti-cogging (2026-11). The capture and histogram settings are RAM only; the anti-cogging
// mode and scale are saved in block 3 with the harmonics and the table. R = read-only.
#define PARAM_CAPTURE_SOURCE_A 24   // signal of channel A (SIGNAL_*), also the trigger channel
#define PARAM_CAPTURE_SOURCE_B 25   // signal of channel B
#define PARAM_CAPTURE_SOURCE_C 26   // signal of channel C
#define PARAM_CAPTURE_DIVIDER 27    // control loops per sample, 1-65535
#define PARAM_CAPTURE_TRIGGER 28    // writing starts (TRIGGER_NOW..FALLING) or stops (TRIGGER_STOP) a capture
#define PARAM_CAPTURE_LEVEL 29      // trigger level, in channel A's units
#define PARAM_CAPTURE_PRETRIGGER 30 // samples kept from before the trigger, 0 to the capture length - 1
#define PARAM_CAPTURE_STATE 31      // R CAPTURE_IDLE, ARMED, RECORDING, DONE
#define PARAM_CAPTURE_COUNT 32      // R samples readable, 0 to the capture length
#define PARAM_CAPTURE_TRIGGER_INDEX 33 // R the trigger sample's index in the read-out, -1 if not triggered
#define PARAM_HIST_X 34             // writing HIST_X_ELECTRICAL or HIST_X_ENCODER clears and starts; HIST_X_OFF stops
#define PARAM_HIST_Y 35             // the signal averaged per bin (SIGNAL_*); set it before PARAM_HIST_X
#define PARAM_HIST_SAMPLES 36       // R samples summed since the start; stops by itself when a bin holds 65535
#define PARAM_COGGING_MODE 37       // anti-cogging: 0 off, 1 harmonics, 2 table (velocity and position modes)
#define PARAM_COGGING_SCALE 38      // percent of the stored anti-cogging applied, 0-200
#define PARAM_SENSE1_OFFSET_OFF 40  // R current-sense zero 1 measured with the bridge off, counts from 2048
#define PARAM_SENSE2_OFFSET_OFF 41  // R current-sense zero 2 measured with the bridge off
#define PARAM_SENSE_ZERO_SWITCHING 42 // R the zero measured with the bridge switching: SENSE_ZERO_SW_*
#define PARAM_SENSE_ZERO_USE_SWITCHING 46 // 1: use the zero measured with the bridge switching, 0: the bridge-off
                                    // zero (default; the switching zero raised the 1st harmonic on the bench)
#define PARAM_SENSE1_OFFSET_SW 47   // R current-sense zero 1 measured with the bridge switching, counts
#define PARAM_SENSE2_OFFSET_SW 48   // R current-sense zero 2 measured with the bridge switching
#define PARAM_CURRENT_BANDWIDTH 49  // current-loop bandwidth, Hz: writing sets Kp_iq/Ki_iq at once from the
                                    // measured R and L (RAM; #Save keeps the gains); reading gives the
                                    // bandwidth the gains in force amount to (0: R, L not measured)
#define CURRENT_BANDWIDTH_DEFAULT 160 // Hz (1000 rad/s, the usual default for such a loop; below loop rate / 30)
#define CURRENT_BANDWIDTH_MIN 10
#define CURRENT_BANDWIDTH_MAX (LOOP_FREQ / 10)
#define PARAM_SPEED_ESTIMATOR 50    // the speed the loops use: SPEED_ESTIMATOR_* (RAM)
#define SPEED_ESTIMATOR_AVERAGE 0   // the 20-sample moving average of the count difference (as before)
#define SPEED_ESTIMATOR_PLL 1       // a PLL whose speed is its integrator (smooth, lags)
#define SPEED_ESTIMATOR_PLL_DIRECT 2 // the same PLL, speed = integrator + kp x error (no lag, more noise)
#define SPEED_ESTIMATOR_DEFAULT SPEED_ESTIMATOR_AVERAGE
#define PARAM_SPEED_PLL_BANDWIDTH 51 // the PLLs' natural frequency, Hz (RAM)
#define SPEED_PLL_BANDWIDTH_DEFAULT 160 // Hz: 1000 rad/s, a usual default (the STEPFOC uses 60 since its bench)
#define SPEED_PLL_BANDWIDTH_MIN 10
#define SPEED_PLL_BANDWIDTH_MAX (LOOP_FREQ / 20)
#define SPEED_PLL_JUMP (CPR / 2)    // counts in one loop that are a position reset, not motion
// The encoder table (encal.h), batch 9. Parameter 52 runs it; 43 or 52 = 3 runs it, then the anti-cogging, saves both.
#define PARAM_ENCAL 52              // write 1: measure the encoder table and use it, 2: and save, 3: then the
                                    // anti-cogging too and save both, 0: abort; read: ENCAL_*
#define PARAM_ENCAL_USE 53          // 1: correct the encoder with the table (only if one was measured)
#define PARAM_ENCAL_PP 54           // R the table's error peak to peak, 0.01 deg of the shaft
#define PARAM_ENCAL_ONCE 55         // R its once-per-turn part (amplitude), 0.01 deg of the shaft
#define PARAM_AFTER_CAL 56          // what runs after a successful calibration: AFTER_CAL_* (saved with the table)
#define PARAM_ENCAL_OFFSET 57       // R how far the last run moved the electrical offset, 0.1 deg electrical
#define PARAM_ID_SETPOINT 58        // the d-axis current setpoint, mA (RAM, as #Id): no torque, more phase current
// The current-sense zero trim (batch 11): parameter 43 = 4 identifies the anti-cogging, corrects the zeros from its
// 1st harmonic and identifies again; 5 also saves. The trims are added to the zeros measured at every start.
#define PARAM_ZERO_TRIM1 59         // the trim of current-sense zero 1, counts (saved with the encoder block)
#define PARAM_ZERO_TRIM2 60         // the trim of zero 2
#define PARAM_THETA_OFFSET 61       // R the electrical offset, 0.1 deg electrical
#define PARAM_TRIM_H1_BEFORE 62     // R the 1st harmonic before the last trim, 0.1 mA
#define PARAM_TRIM_H1_AFTER 63      // R and after it, 0.1 mA
#define PARAM_COMM_DELAY_US 64      // commutation delay compensation, us (0 = off, RAM; batch 12)
#define COMM_DELAY_MAX_US 500
#define COMM_DELAY_MIN_SPEED 100   // ticks/s: below it no rotation (under 0.02 deg electrical; batch 17)
#define PARAM_FAST_TRIG 65          // 1: sin/cos of the transforms from the integer table, 0: qfplib (default)
// The velocity-loop details (batch 18), RAM, 0 = off (the default). 73 (an anti-windup decay) was removed in batch 19.
#define PARAM_INPUT_FILTER_HZ 74    // the position input filter, true Hz, critically damped; position and PD
                                    // modes; about the host's setpoint rate / 12;
                                    // for hosts that send positions without the speed (bench 2026-11-08)
#define PARAM_VEL_INT_DEADBAND 75   // ticks: the velocity integral sees the position error less this band
// The motor (batch 20), as #R, #L, #PP; saved by #Save. The current gains follow at the next calibration (or 49).
#define PARAM_R_MOHM 76             // phase resistance, milliohm (1-100000)
#define PARAM_L_UH 77               // phase inductance, microhenry (1-1000000)
#define PARAM_POLE_PAIRS 78         // pole pairs (1-200), only while idle; calibrate after a change
#define PARAM_FEATURES 79           // R what is in force and what runs: the bits FEAT_* (batch 20)
#define FEAT_CALIBRATED (1u << 0)
#define FEAT_ENCODER_MEASURED (1u << 1) // an encoder table is in RAM
#define FEAT_ENCODER_IN_USE (1u << 2)
#define FEAT_COGGING_SET (1u << 3)      // an anti-cogging set is in RAM (harmonics or table)
#define FEAT_COGGING_ON (1u << 4)
#define FEAT_COGGING_TABLE (1u << 5)    // on as the table (otherwise the harmonics)
#define FEAT_COGGING_SAVED (1u << 6)    // the set in RAM is the one in EEPROM
#define FEAT_ZERO_TRIM (1u << 7)        // a current-sense zero trim is applied
#define FEAT_ZERO_SWITCHING (1u << 8)   // the zero measured with the bridge switching is in use
#define FEAT_DELAY_COMP (1u << 9)       // commutation delay compensation (parameter 64 not 0)
#define FEAT_SPEED_PLL (1u << 10)       // the speed estimate is a PLL (otherwise the moving average)
#define FEAT_FAST_READ (1u << 11)       // STEPFOC
#define FEAT_FAST_PWM (1u << 12)        // STEPFOC
#define FEAT_FAST_TRIG (1u << 13)       // sin/cos from the table
#define FEAT_INPUT_FILTER (1u << 14)
#define FEAT_DEADBAND (1u << 15)
#define FEAT_AFTER_CAL_SHIFT 16         // bits 16-17: parameter 56
#define FEAT_RUN_CALIBRATION (1u << 20) // a calibration is running
#define FEAT_RUN_ENCODER (1u << 21)     // the encoder table is being measured
#define FEAT_RUN_COGGING (1u << 22)     // the anti-cogging is being identified
#define FEAT_RUN_PENDING (1u << 23)     // the runs after a calibration are about to start
#define FEAT_RUN_SOUND (1u << 24)       // sound is playing
// Sound (batch 22, sound.h)
#define PARAM_SOUND_STATE 80        // R bits 0-7 queued events, 8-18 buffered samples, 24 tones, 25 queue, 26 samples,
                                    // 27 the samples ran dry
#define PARAM_SOUND_LIMIT_MV 81     // the sound's voltage at most, mV (0 refuses sound; default SOUND_LIMIT_DEFAULT_MV)
#define PARAM_SOUND_AXIS 82         // 1: on the q axis (a small torque ripple; the default, much louder), 0: the d axis
#define SOUND_LIMIT_DEFAULT_MV 2000
#define SOUND_LIMIT_MAX_MV 6000
#define INPUT_FILTER_MAX_HZ (LOOP_FREQ / 25)
#define VEL_INT_DEADBAND_MAX 2000
#define ENCAL_IDLE 0
#define ENCAL_FORWARD 1             // the field turns forward (after a 0.3 s current ramp)
#define ENCAL_BACKWARD 2
#define ENCAL_DONE 3                // the table and the moved offset are in use (saved if asked)
#define ENCAL_FAILED_ABORTED 4      // a command from a host came first (or #Encal stop)
#define ENCAL_FAILED_ERROR 5        // the drive was or went into error, or is not calibrated
#define ENCAL_FAILED_STUCK 6        // the rotor did not follow the field (blocked, or too little current)
#define ENCAL_REJECTED 7            // implausible: over 4 deg peak to peak, or an offset move over 60 deg electrical
#define AFTER_CAL_NOTHING 0
#define AFTER_CAL_COGGING 1         // the anti-cogging identification
#define AFTER_CAL_BOTH 2            // the encoder table, then the anti-cogging
#define AFTER_CAL_DEFAULT AFTER_CAL_NOTHING // until the Spectral bench (STEPFOC: both, batch 20)
#define MODE_ENCAL 12               // Controller_mode while the encoder table is measured (open loop)
#define ENCAL_TICKS_PER_S 2048      // the field's speed: one shaft turn in 8 s
#define ENCAL_RAMP_MS 300           // the current rises at the start angle first
#define ENCAL_HARMONICS 16          // mechanical harmonics fitted
#define PARAM_COGCAL 43             // write 1: identify the anti-cogging on the drive, 2: and save, 0: abort; read: COGCAL_*
#define PARAM_COGCAL_SPEED 44       // ticks/s the identification turns at, 100-20000 (1500)
#define PARAM_COGCAL_RIPPLE 45      // R the identified cogging current, rms, 0.1 mA
#define COGCAL_IDLE 0
#define COGCAL_FORWARD 1            // turning forward, then recording one motor turn
#define COGCAL_BACKWARD 2
#define COGCAL_DONE 3               // harmonics and table stored, harmonics on (saved if asked)
#define COGCAL_FAILED_ABORTED 4     // a command from a host came first (or #Cogcal stop)
#define COGCAL_FAILED_ERROR 5       // the drive was or went into error, or is not calibrated
#define COGCAL_FAILED_STUCK 6       // the shaft did not turn through whole electrical turns
#define COGCAL_ENCODER 7            // a combined run (43 or 52 = 3) is measuring the encoder table first
#define COGCAL_FAILED_ENCODER 8     // the encoder table of a combined run failed (parameter 52 says why)
#define COGCAL_FAILED_TRIM 9        // the current-sense trim did not lower the 1st harmonic: zeros put back
#define COGCAL_STALE 10             // switched off: the encoder table was measured again on its own; identify again
#define COGCAL_DONE_NO_TRIM 11      // done, harmonics on, without the trim (it did not lower the 1st harmonic)
#define COGCAL_TRIM_MIN_MA 1.5f     // a 1st harmonic below this is left alone
#define COGCAL_TRIM_MAX 30          // counts one trim run may move a zero
#define ZERO_TRIM_MAX 60            // counts a stored trim may reach
#define PARAM_CAPTURE_LENGTH 39     // R samples per channel a capture holds (SCOPE_LEN, 1024 since 2026-11-07)

// INPUT CAN command IDS (Commands that spectral driver can receive)
// To these commands spectral responds with specific command ID
#define IN_DATA_PACK_1 2
#define IN_DATA_PACK_2 6
#define IN_DATA_PACK_3 8
#define IN_DATA_PACK_PD 4


// GRIPPER SPECIFIC COMMANDS
// OUTPUT CAN command IDs (Commands that gripper can send)
#define OUT_GRIPPER_DATA_PACK 60

// INPUT CAN command IDs (Commands that gripper can receive)
#define IN_GRIPPER_DATA_PACK 61
#define IN_GRIPPER_CALIB 62


#endif