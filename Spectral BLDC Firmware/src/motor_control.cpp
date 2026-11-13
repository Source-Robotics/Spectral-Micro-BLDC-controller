/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    motor_control.cpp
 * @brief   This file provides code for utilities for motor control
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "utils.h"
#include <Arduino.h>
#include <stdio.h>
#include "iodefs.h"
#include "hw_init.h"
#include "motor_control.h"
#include "common.h"
#include <SPI.h>
#include "common.h"
#include "foc.h"
#include <IWatchdog.h>
#include "watchdog_action.h"
#include "cogging.h"
#include "scope.h"
#include "encal.h"
#include "sound.h"

/// Current-sense sampling point (2026-11-09). The sense amplifiers are read by a software start, wherever the code
/// happens to be in the 25 kHz centre-aligned PWM cycle of TIM2, so the moment of a reading depends on the code before
/// it. Parameter 88: -1 reads as before; 0 to the PWM period (TIM2 ARR) waits until the PWM counter is near that
/// count, then reads. Parameter 89 reports where the last reading happened (the count, +65536 while counting down).
volatile int32_t Sense_sample_at = -1;
volatile int32_t Sense_sample_seen = 0;
static inline void Sense_sample_wait()
{
  const int32_t target = Sense_sample_at;
  if (target >= 0)
  {
    for (int guard = 0; guard < 300; guard++) // at most about 60 us: the counter passes every count twice per period
    {
      const int32_t cnt = (int32_t)TIM2->CNT;
      if (cnt >= target - 24 && cnt <= target + 24)
        break;
    }
  }
  Sense_sample_seen = (int32_t)TIM2->CNT | ((TIM2->CR1 & TIM_CR1_DIR) ? 0x10000 : 0);
}

#define TIMING_DEBUG 0

SPISettings MT6816_settings(16000000, MSBFIRST, SPI_MODE3);

/// @todo phase order working in any orientation
/// @todo feedforwards for cascade control and PD control
/// @todo cogging compensation
/// @todo encoder non linearity compensation

/// @brief  Collect all sensor data
void Collect_data()
{

  uint16_t result;
  uint16_t result1;
  uint16_t result2;

  /*Get position, current1, current2, vbus*/
  /*position sensor packs its data in 2 8 bit data packets */
  /*First we send 0x8300 as a response we get one packet*/
  /*Then we send 0c8400 as a response we get second packet*/
  /*Between requesting packets we need small delay and we use ADC readings as a delay*/
  /*Reading sense1,2 and vbus takes around 10us */
  /*After we get both packets they are unpacked and we get position value as 14 bit, no mag bit and parity check bit */
  SPI.beginTransaction(MT6816_settings);
  digitalWriteFast(CSN, LOW);
  result1 = SPI.transfer16(0x8300);
  digitalWriteFast(CSN, HIGH);

  Sense_sample_wait();
  if (controller.DIR_ == 1)
  {
    controller.Sense1_Raw = ADC_CHANNEL_4_READ_SENSE1() - controller.sense1_offset;
    controller.Sense2_Raw = ADC_CHANNEL_3_READ_SENSE2() - controller.sense2_offset;
  }
  else
  {
    controller.Sense1_Raw = ADC_CHANNEL_3_READ_SENSE2() - controller.sense2_offset;
    controller.Sense2_Raw = ADC_CHANNEL_4_READ_SENSE1() - controller.sense1_offset;
  }

  controller.VBUS_RAW = ADC_CHANNEL_6_READ_VBUS();
  result1 = result1 << 8;
  digitalWriteFast(CSN, LOW);
  result2 = SPI.transfer16(0x8400);
  digitalWriteFast(CSN, HIGH);
  result = result1 | result2;
  controller.Position_Raw = Encoder_correct(result >> 2); // the encoder table, when in use
  SPI.endTransaction();
  /***********************************/

  /* Check for magnet on MT6816*/
  if ((result & MT6816_NO_MAGNET_BIT) == MT6816_NO_MAGNET_BIT)
  { // no magnet or not detecting enough flux density
    controller.Magnet_warrning = true;
    controller.encoder_error = 1;
  }
  else
  { // Detected magnet
    controller.Magnet_warrning = false;
  }
  /***********************************/

  /* Parity check, works wierd and inconsistent, skip for now
 if(parityCheck(result)){
    controller.MT6816_parity_check_pass = 1;
 }else{
    controller.MT6816_parity_check_pass = 0;
 }
*/

  /* Transform RAW current and voltage to mV, mA.*/
  /* Ia + Ib + Ic = 0. Calculate Sense3 .*/

  controller.Sense1_mA = -Get_current_mA(controller.Sense1_Raw);
  controller.Sense2_mA = -Get_current_mA(controller.Sense2_Raw);
  controller.Sense3_mA = -controller.Sense1_mA - controller.Sense2_mA;

  // Reversed-wiring compensation (software reflection): swap phase-B/phase-C currents
  // (Sense2 <-> Sense3) when commutation_dir < 0, matching the PWM2<->PWM3 swap in Phase_order().
  // Kirchhoff still holds (Sense1+Sense2+Sense3 == 0). This turns a reversed-wired motor into a
  // normally-wired (proper-rotation) one so it calibrates/runs like the forward wiring.
  // (bench-confirm: B<->C is the conventional 3-phase reflection axis)
  if (controller.commutation_dir < 0)
  {
    int tmp = controller.Sense2_mA;
    controller.Sense2_mA = controller.Sense3_mA;
    controller.Sense3_mA = tmp;
  }

  controller.VBUS_mV = Get_voltage_mV(controller.VBUS_RAW);
  /***********************************/

  /* Handle encoder full rotation overflow*/
  if (controller.Position_Raw - controller.Old_Position_Raw > CPR2)
  {
    controller.ROTATIONS = controller.ROTATIONS - 1;
  }
  else if (controller.Position_Raw - controller.Old_Position_Raw < -CPR2)
  {
    controller.ROTATIONS = controller.ROTATIONS + 1;
  }
  /***********************************/

  /* Motor position with multiple rotations in encoder ticks*/
  controller.Position_Ticks = controller.Position_Raw + (controller.ROTATIONS * CPR);
  /***********************************/

  /* Calculate velocity in ticks/s */
  controller.Velocity = (controller.Position_Ticks - controller.Old_Position_Ticks) * LOOP_FREQ;
  /* The speed the loops use: the moving average or the PLL (parameter 50) */
  controller.Velocity_Filter = Speed_estimate(controller.Velocity, controller.Position_Ticks - controller.Old_Position_Ticks);
  /***********************************/

  /*  Save position values for next cycle*/
  controller.Old_Position_Ticks = controller.Position_Ticks;
  controller.Old_Position_Raw = controller.Position_Raw;
  /***********************************/

  // Position_RAD = qfp_fmul(Position_Raw, RAD_CONST);

  /* Caclculate electrical angle */
  // controller.Electric_Angle = RAD_CONST * fmod((controller.Position_Raw * NPP),CPR);
  controller.Electric_Angle = RAD_CONST * ((controller.Position_Raw * controller.pole_pairs) % CPR);

  // Add offset
  controller.Electric_Angle += controller.theta_offset;

  // Normalize angle between 0 and 2Pi
  if (controller.Electric_Angle < 0)
    controller.Electric_Angle = (controller.Electric_Angle + PI2);
  else if (controller.Electric_Angle > PI2)
    controller.Electric_Angle = (controller.Electric_Angle - PI2);
  /***********************************/

  /****  End measurment here *********/
}

void Get_first_encoder()
{
  uint16_t result;
  uint16_t result1;
  uint16_t result2;

  /*Get position, current1, current2, vbus*/
  /*position sensor packs its data in 2 8 bit data packets */
  /*First we send 0x8300 as a response we get one packet*/
  /*Then we send 0c8400 as a response we get second packet*/
  /*Between requesting packets we need small delay and we use ADC readings as a delay*/
  /*Reading sense1,2 and vbus takes around 10us */
  /*After we get both packets they are unpacked and we get position value as 14 bit, no mag bit and parity check bit */
  SPI.beginTransaction(MT6816_settings);
  digitalWriteFast(CSN, LOW);
  result1 = SPI.transfer16(0x8300);
  digitalWriteFast(CSN, HIGH);

  delayMicroseconds(100);
  result1 = result1 << 8;
  digitalWriteFast(CSN, LOW);
  result2 = SPI.transfer16(0x8400);
  digitalWriteFast(CSN, HIGH);
  result = result1 | result2;
  controller.Position_Raw = Encoder_correct(result >> 2); // the encoder table, when in use
  SPI.endTransaction();
  /***********************************/

  /* Check for magnet on MT6816*/
  if ((result & MT6816_NO_MAGNET_BIT) == MT6816_NO_MAGNET_BIT)
  { // no magnet or not detecting enough flux density
    controller.Magnet_warrning = true;
    controller.encoder_error = 1;
  }
  else
  { // Detected magnet
    controller.Magnet_warrning = false;
  }
  /***********************************/

  /* Motor position with multiple rotations in encoder ticks*/
  controller.Position_Ticks = controller.Position_Raw;
  /***********************************/

  /*  Save position values for next cycle*/
  controller.Old_Position_Ticks = controller.Position_Ticks;
  controller.Old_Position_Raw = controller.Position_Raw;
  /***********************************/
}

/// The gate driver awake: nSLEEP and nRESET high. Written every time, never only "when the flags say asleep".
/// The command handlers in the main loop wake the driver too, in two steps (the pins, then the flags); an idle loop of
/// this interrupt landing between the two put the pins low again and the handler then set the flags to "awake". From
/// then on every wake was skipped and the motor got no current, with no error, until the next idle (2026-11-10).
/// So the control loop owns the pins: every loop of a driving mode calls this, every idle loop calls Phases_coast().
static inline void Driver_wake()
{
  digitalWriteFast(SLEEP, HIGH);
  digitalWriteFast(RESET, HIGH);
  controller.reset_pin_state = 1;
  controller.sleep_pin_state = 1;
}

/// The current loop is not running (idle, brake, coast): its integrals start from zero the next time, and a bad
/// number (NaN) cannot stay in them until the next power cycle.
static inline void Current_integrals_clear()
{
  PID.Iq_errSum = 0.0f;
  PID.Id_errSum = 0.0f;
}

/// Brake: phases shorted through the driver (driver stays enabled). NOT current-limited -- see the note in the body.
static void Phases_brake()
{
  /// Brake: keep the driver enabled and hold the zero-voltage PWM vector (all
  /// three phases at the same duty cycle), which shorts the windings together
  /// through the driver's complementary switches. Back-EMF drives the braking
  /// current -- it is not actively current-limited here, so only enable this on
  /// motors where phase resistance keeps that current within MAX_DRIVE_CURRENT
  /// at the speeds this board is actually spinning at when brake is commanded.
  Driver_wake();
  Current_integrals_clear();
  pwm_set(PWM_CH1, PWM_MAX / 2, 13);
  pwm_set(PWM_CH2, PWM_MAX / 2, 13);
  pwm_set(PWM_CH3, PWM_MAX / 2, 13);
}

/// Coast: driver disabled, outputs Hi-Z.
static void Phases_coast()
{
  /// Coast: disable the driver entirely (Hi-Z outputs, freewheel).
  /// Sleep mode input.Logic high to enable device;
  /// logic low to enter low-power sleep mode; internal pulldown
  digitalWriteFast(SLEEP, LOW);
  /// Reset input. Active-low reset input initializes internal logic, clears faults,
  /// and disables the outputs, internal pulldown
  digitalWriteFast(RESET, LOW);
  controller.reset_pin_state = 0;
  controller.sleep_pin_state = 0;
  Current_integrals_clear();
}

/// True when the CAN watchdog is the ONLY thing wrong. Mirrors the flags IT_callback() folds into
/// controller.Error -- a new error source added there must be added here too, otherwise it would get
/// the gentler watchdog response instead of the full one.
static bool only_watchdog_error()
{
  if (controller.temperature_error || controller.encoder_error || controller.Vbus_error ||
      controller.Velocity_error || controller.Driver_error || controller.Current_error ||
      controller.ESTOP_error || controller.Calibrated == 0)
    return false;
  if (controller.I_AM_GRIPPER == 1 && (Gripper.estop_error || Gripper.temperature_error))
    return false;
  return controller.Watchdog_error == 1;
}

/// Current-sense zero (2026-11). The firmware assumed the amplifiers read exactly 2048 at zero current; real boards
/// are a few counts off (bench: about 48 mA of Iq on STEPFOC, -20 to +19 mA on Spectral with nothing flowing). The
/// current loop then holds that error as real current, and because the error turns with the rotor it adds a ripple
/// once per electrical cycle. At start-up, with the bridge off, average SENSE_ZERO_SAMPLES readings of each channel;
/// the offset is subtracted wherever the channel is read. Beyond SENSE_ZERO_MAX_DEV counts it is not trusted (a
/// fault, or current flowing) and 2048 is kept; #Info and the identity flags say which.
void Sense_zero_calibrate()
{
  Phases_coast(); // driver asleep, outputs Hi-Z
  // The current-sense amplifiers need the motor supply: without it (USB plugged in first) they read 50-70 counts
  // off, about 100 mA, and that zero was used as if it were right (bench 2026-11-10). So wait a little for a supply
  // that is still coming up; if it does not come, 2048 is used until the control loop measures the zero itself
  // (Sense_zero_late_step(), as soon as the supply is there).
  const uint32_t t_wait = millis();
  int vbus_mv = Get_voltage_mV(ADC_CHANNEL_6_READ_VBUS());
  while (vbus_mv < controller.Min_Vbus && millis() - t_wait < SENSE_ZERO_VBUS_WAIT_MS)
  {
    delay(10);
    vbus_mv = Get_voltage_mV(ADC_CHANNEL_6_READ_VBUS());
  }
  delay(20);
  int32_t sum1 = 0;
  int32_t sum2 = 0;
  for (int i = 0; i < SENSE_ZERO_SAMPLES; i++)
  {
    sum1 += ADC_CHANNEL_4_READ_SENSE1();
    sum2 += ADC_CHANNEL_3_READ_SENSE2();
    delayMicroseconds(50);
  }
  int off1 = (int)((sum1 + SENSE_ZERO_SAMPLES / 2) / SENSE_ZERO_SAMPLES) - 2048;
  int off2 = (int)((sum2 + SENSE_ZERO_SAMPLES / 2) / SENSE_ZERO_SAMPLES) - 2048;
  controller.sense_zero_wait_vbus = (vbus_mv < controller.Min_Vbus);
  controller.sense_zero_ok = !controller.sense_zero_wait_vbus && abs(off1) <= SENSE_ZERO_MAX_DEV && abs(off2) <= SENSE_ZERO_MAX_DEV;
  // plus the trims an anti-cogging run found (encal.h; 0 until one ran): the bridge-off zeros are a few counts off
  controller.sense1_offset = controller.sense_zero_ok ? off1 + encoder.zero_trim[0] : 0;
  controller.sense2_offset = controller.sense_zero_ok ? off2 + encoder.zero_trim[1] : 0;
  controller.sense1_offset_off = controller.sense1_offset;
  controller.sense2_offset_off = controller.sense2_offset;
  controller.sense_zero_switching = SENSE_ZERO_SW_RUNNING; // the control loop measures it again with the bridge switching
}

/// Current-sense zero with the bridge switching (2026-11). The start-up zero is measured with the bridge off, but in
/// motion the readings are taken with the bridge switching, which can shift them by a few counts: a constant error
/// that the current loop turns into a ripple at the 1st harmonic of the electrical angle. So the first idle loops
/// after start-up hold the zero-voltage vector at 50 % duty (no current flows into a motor at rest), wait
/// SENSE_ZERO_SW_SETTLE loops, then average SENSE_ZERO_SW_SAMPLES readings of each channel exactly where the control
/// loop reads them; a plausible result is kept for parameter 46 to switch in. Returns true while it owns the bridge.
static uint16_t sw_zero_loops = 0;
static int32_t sw_zero_sum1 = 0;
static int32_t sw_zero_sum2 = 0;
static bool Sense_zero_switching_step()
{
  if (controller.sense_zero_switching != SENSE_ZERO_SW_RUNNING)
    return false;
  Phases_brake(); // the zero-voltage vector: all three phases at 50 % duty, driver awake
  if (sw_zero_loops >= SENSE_ZERO_SW_SETTLE)
  {
    // The readings as Collect_data() took them, before the zero in use was subtracted. Zero 1 belongs to ADC channel
    // 4 and zero 2 to channel 3; which of Sense1/Sense2 holds which follows DIR_ (see Collect_data()).
    if (controller.DIR_ == 1)
    {
      sw_zero_sum1 += controller.Sense1_Raw + controller.sense1_offset;
      sw_zero_sum2 += controller.Sense2_Raw + controller.sense2_offset;
    }
    else
    {
      sw_zero_sum1 += controller.Sense2_Raw + controller.sense1_offset;
      sw_zero_sum2 += controller.Sense1_Raw + controller.sense2_offset;
    }
  }
  sw_zero_loops++;
  if (sw_zero_loops <= SENSE_ZERO_SW_SETTLE + SENSE_ZERO_SW_SAMPLES)
    return true;
  const int o1 = (int)((sw_zero_sum1 + SENSE_ZERO_SW_SAMPLES / 2) / SENSE_ZERO_SW_SAMPLES) - 2048;
  const int o2 = (int)((sw_zero_sum2 + SENSE_ZERO_SW_SAMPLES / 2) / SENSE_ZERO_SW_SAMPLES) - 2048;
  bool ok = abs(o1) <= SENSE_ZERO_MAX_DEV && abs(o2) <= SENSE_ZERO_MAX_DEV;
  if (controller.sense_zero_ok)
    ok = ok && abs(o1 - controller.sense1_offset_off) <= SENSE_ZERO_SW_MAX_SHIFT &&
         abs(o2 - controller.sense2_offset_off) <= SENSE_ZERO_SW_MAX_SHIFT;
  if (ok)
  {
    // Kept, not used: on the bench (2026-11-07) the switching zero raised the 1st harmonic; parameter 46 switches it in.
    controller.sense1_offset_sw = o1 + encoder.zero_trim[0]; // with the stored trim, as the bridge-off zeros
    controller.sense2_offset_sw = o2 + encoder.zero_trim[1];
    controller.sense_zero_switching = SENSE_ZERO_SW_MEASURED;
  }
  else
  {
    controller.sense_zero_switching = SENSE_ZERO_SW_REJECTED;
  }
  return false;
}

/// The bridge-off zero, measured by the control loop when there was no motor supply at start-up (see
/// Sense_zero_calibrate()). It waits until the supply has been there, and the driver asleep, for SENSE_ZERO_LATE_SETTLE
/// loops in a row (idle, or the error branch: the Vbus error a late supply latches), then averages SENSE_ZERO_SAMPLES
/// readings as Collect_data() took them, and starts the switching zero again, which had no supply either.
/// Anything else in between (a command, the supply dropping) starts the count again.
static void Sense_zero_late_step()
{
  static uint16_t loops = 0;
  static int32_t sum1 = 0, sum2 = 0;
  if (!controller.sense_zero_wait_vbus)
    return;
  if (controller.VBUS_mV < controller.Min_Vbus || controller.sleep_pin_state != 0 || controller.Controller_mode != 0)
  {
    loops = 0;
    sum1 = sum2 = 0;
    return;
  }
  loops++;
  if (loops <= SENSE_ZERO_LATE_SETTLE)
    return;
  // the readings before the zero in use was subtracted; zero 1 belongs to ADC channel 4, zero 2 to channel 3
  if (controller.DIR_ == 1)
  {
    sum1 += controller.Sense1_Raw + controller.sense1_offset;
    sum2 += controller.Sense2_Raw + controller.sense2_offset;
  }
  else
  {
    sum1 += controller.Sense2_Raw + controller.sense1_offset;
    sum2 += controller.Sense1_Raw + controller.sense2_offset;
  }
  if (loops < SENSE_ZERO_LATE_SETTLE + SENSE_ZERO_SAMPLES)
    return;
  const int off1 = (int)((sum1 + SENSE_ZERO_SAMPLES / 2) / SENSE_ZERO_SAMPLES) - 2048;
  const int off2 = (int)((sum2 + SENSE_ZERO_SAMPLES / 2) / SENSE_ZERO_SAMPLES) - 2048;
  controller.sense_zero_ok = (abs(off1) <= SENSE_ZERO_MAX_DEV && abs(off2) <= SENSE_ZERO_MAX_DEV);
  controller.sense1_offset = controller.sense_zero_ok ? off1 + encoder.zero_trim[0] : 0;
  controller.sense2_offset = controller.sense_zero_ok ? off2 + encoder.zero_trim[1] : 0;
  controller.sense1_offset_off = controller.sense1_offset;
  controller.sense2_offset_off = controller.sense2_offset;
  controller.sense_zero_wait_vbus = 0;
  loops = 0;
  sum1 = sum2 = 0;
  sw_zero_loops = 0; // and the switching zero, measured again in the next idle loops
  sw_zero_sum1 = sw_zero_sum2 = 0;
  controller.sense_zero_switching = SENSE_ZERO_SW_RUNNING;
}

/// Watchdog fallback (hold / descend, see watchdog_action.h). Started on the first control loop of a
/// pure watchdog trip, if the drive was in a closed-loop mode; cleared as soon as the error is cleared.
static bool wd_fallback_active = false;
// An error is latched and its first loop is over. Hold and descend are decided on that first loop only: a trip
// while idle started them later, when the next motion frame set a mode with the error still latched.
static bool wd_error_seen = false;
static uint32_t wd_fallback_loops = 0; // control loops since the fallback started
static int wd_hold_position = 0;       // position captured at the trip, held by WD_RESP_HOLD

/// Modes the fallback may take over from: position, velocity, torque, PD, gripper.
static bool watchdog_fallback_mode_ok(int mode)
{
  return mode == 1 || mode == 2 || mode == 3 || mode == 4 || mode == 6;
}

/// @brief What a motion frame does to the bridge before it moves: wake the gate driver if nothing is wrong.
void Motion_wake()
{
  if (controller.Error == 0)
    Driver_wake();
}

/// Descend: the fallback current, with the speed held to +-wd_fallback_speed. A PI acts only on the
/// speed beyond the cap. Inside the cap its integral relaxes back to zero (time constant
/// WD_DESCEND_RELAX_S), so it only ever holds what keeps the speed at the cap: a joint under gravity
/// descends at the cap and rests once something stops it, a gripper keeps exactly the fallback
/// current, and a free motor turns at about the cap speed.
/// Bench 2026-11-07, before the relaxation: the step at the trip spiked a bare rotor to 5000 ticks/s,
/// the integral wound to about -320 mA to stop it and then kept that value, cancelling a 200 mA
/// fallback current and parking the motor.
#define WD_DESCEND_RELAX_S 0.25f
static const float WD_DESCEND_RELAX = 1.0f - 1.0f / ((float)LOOP_FREQ * WD_DESCEND_RELAX_S);
static void Watchdog_descend()
{
  const float v = controller.Velocity_Filter;
  const float cap = (float)controller.wd_fallback_speed;
  float excess = 0;
  if (v > cap)
    excess = v - cap;
  else if (v < -cap)
    excess = v + cap;

  if (excess == 0.0f)
    PID.V_errSum = PID.V_errSum * WD_DESCEND_RELAX;
  else
    PID.V_errSum = PID.V_errSum - excess * PID.Ki_v;
  if (PID.V_errSum > PID.Iq_current_limit)
    PID.V_errSum = PID.Iq_current_limit;
  else if (PID.V_errSum < -PID.Iq_current_limit)
    PID.V_errSum = -PID.Iq_current_limit;

  PID.Iq_setpoint = (float)controller.wd_fallback_current - PID.Kp_v * excess + PID.V_errSum;
  Torque_mode(); // clamps to the current limit
}

/// @brief Interrupt callback routine for FOC mode
void IT_callback(void)
{
  IWatchdog.reload();

  // Always measured (cheap: 2x micros() + subtract) so execution_time is available live
  // via #Info without needing to recompile with TIMING_DEBUG.
  int c = micros();

  /*Sample temperature every 15000 ticks; If enabled*/
  if (controller.Thermistor_on_off == 1)
  {
    static int temperature_tick = 0;
    if (temperature_tick >= 15000)
    {
      controller.TEMP_RAW = ADC_CHANNEL_5_READ_TEMP() >> 4;
      controller.TEMP_DEG = Temp_table[controller.TEMP_RAW];
      temperature_tick = 0;
    }
    temperature_tick = temperature_tick + 1;
  }
  /***********************************/

  /* Get all data needed for FOC */
  Collect_data();
  /***********************************/

  /* Calculate Iq and Id currents */
  dq0_abc_variables(controller.Electric_Angle);
  dq0_fast_int(controller.Sense1_mA, controller.Sense2_mA, controller.Sense3_mA, &FOC.Id, &FOC.Iq);

  /***********************************/

  /* If motor controller is gripper */
  if (controller.I_AM_GRIPPER == 1)
  {
    Gripper.temperature_error = controller.temperature_error;
    Gripper.timeout_error = controller.Watchdog_error;
    Gripper.position_ticks = controller.Position_Raw - 16383;
    if (Gripper.estop_status == 1)
    {
      Gripper.estop_error = 1;
      controller.Error = 1;
    }

    Gripper.position = mapAndConstrain(controller.Position_Ticks, Gripper.max_closed_position, Gripper.max_open_position);
  }
  /***********************************/

  /* Check for errors, If errors are present go idle  */

  ///  Check termistor
  if (controller.Thermistor_on_off == 1)
  {
    if (controller.TEMP_DEG < controller.Min_temperature || controller.TEMP_DEG > controller.Max_temperature)
    {
      controller.temperature_error = 1;
      controller.Error = 1;
    }
  }
  /// Check Vbus
  /// Undervoltage only after the start-up grace: the supply may still be ramping when the board boots, which
  /// used to latch a Vbus error at every power-up. Overvoltage is always checked.
  static uint32_t vbus_grace_loops = (uint32_t)VBUS_STARTUP_GRACE_MS * LOOP_FREQ / 1000;
  if (vbus_grace_loops > 0)
    vbus_grace_loops--;
  if ((controller.VBUS_mV < controller.Min_Vbus && vbus_grace_loops == 0) || controller.VBUS_mV > controller.Max_Vbus)
  {
    controller.Vbus_error = 1;
    controller.Error = 1;
  }

  /// Encoder error
  if (controller.encoder_error == 1)
  {
    controller.Error = 1;
  }

  /// Driver error
  if (controller.Driver_error == 1)
  {
    controller.Error = 1;
  }

  /// Speed error
  if (abs(controller.Velocity_Filter) > PID.Velocity_limit_error)
  {
    controller.Velocity_error = 1;
    controller.Error = 1;
  }

  /*
  /// Current error; our midpoint is at 0V or 2048 ADC ticks. 0 ADC ticks is -3.5A and 4095 is 3.5A. Our driver can provide around 2.8A
  /// If we go near +- 3.5A report error!
  if (controller.Sense1_Raw > 4090 || controller.Sense1_Raw < 10 || controller.Sense2_Raw > 4090 || controller.Sense2_Raw < 10)
  {
    controller.Current_error = 1;
    controller.Error = 1;
  }
  */

  /// Not calibrated
  if (controller.Calibrated == 0)
  {
    controller.Error = 1;
  }

  /// Estop error
  if (controller.ESTOP_error == 1)
  {
    controller.Error = 1;
  }

  /// Watchdog error
  if (controller.Watchdog_error == 1)
  {
    controller.Error = 1;
  }

  /// The switching zero only runs in idle with nothing wrong; anything else first leaves the bridge-off zero in use.
  if (controller.sense_zero_switching == SENSE_ZERO_SW_RUNNING && (controller.Error || controller.Controller_mode != 0))
    controller.sense_zero_switching = SENSE_ZERO_SW_INTERRUPTED;

  // the position input filter (parameter 74) starts again from the shaft whenever its modes are left
  if (controller.Error || (controller.Controller_mode != 1 && controller.Controller_mode != 4))
    Input_filter_reset();

  // sound (batch 22): the tones and samples for this loop, added to the output on the d (or q) axis, only while the
  // drive holds or moves; idle or an error stops it
  {
    float s = 0.0f;
    if (Sound_active())
    {
      if (controller.Error == 0 && controller.Controller_mode >= 1 && controller.Controller_mode <= 4)
        s = (float)Sound_tick();
      else
        Sound_stop();
    }
    FOC.sound_d = controller.sound_axis ? 0.0f : s;
    FOC.sound_q = controller.sound_axis ? s : 0.0f;
  }

  // A gripper calibration that was cut short: put back what it had changed (Gripper_calibration_abort).
  if (Gripper.In_calibration && (controller.Controller_mode != 7 || controller.Error != 0))
    Gripper_calibration_abort();

  /// If there is no error driver can operate normaly
  /// If error is active we will immediately go to idle
  if (controller.Error == 0)
  {
    wd_fallback_active = false;
    wd_error_seen = false;
    Comm_delay_update(); // parameter 64: the output rotation for this loop
    // every mode that drives the bridge: the driver awake, whatever a handler's wake came to (see Driver_wake())
    // (not gripper mode: it wakes the driver itself while it is active and sleeps it while it is not)
    if ((controller.Controller_mode >= 1 && controller.Controller_mode <= 8 && controller.Controller_mode != 6) ||
        controller.Controller_mode == MODE_ENCAL)
      Driver_wake();
    switch (controller.Controller_mode)
    {
    case 0:
      if (Sense_zero_switching_step())
        break;
      // while a late current-sense zero waits and the supply is there, coast instead of braking (about 80 ms): the
      // zero is measured with the driver asleep (Sense_zero_late_step()); at rest the two are the same for the motor
      if (controller.brake_coast == 1 && !(controller.sense_zero_wait_vbus && controller.VBUS_mV >= controller.Min_Vbus))
      {
        Phases_brake();
      }
      else
      {
        Phases_coast();
      }
      break;
    case 1:
      Position_mode();
      break;
    case 2:
      Velocity_mode();
      break;
    case 3:
      Torque_mode();
      break;
    case 4:
      PD_mode();
      break;
    case 5:
      Open_loop_speed(controller.Open_loop_speed, controller.Open_loop_voltage);
      break;
    case 6:
      Gripper_mode();
      break;
    case 7:
      Calibrate_gripper();
      break;
    case 8:
      Voltage_Torque_mode();
      break;
    case MODE_ENCAL:
      Encal_step(); // the encoder table: open loop on a commanded angle (encal.h)
      break;
    default:
      /// Idle
      break;
    }
  }
  else /// If there is error driver is disabled and in sleep mode
  {
    // Comm_delay_update() runs only without an error: hold and descend used the rotation of the last moving loop
    FOC.delay_c = 1.0f;
    FOC.delay_s = 0.0f;
    /// A pure watchdog trip honours controller.watchdog_action (see watchdog_action.h). Byte 0 (what
    /// RTI and the lib default send), unknown bytes and any other fault keep the old behaviour on
    /// this firmware, which is COAST -- not the persistent brake_coast setting.
    const bool only_wd = only_watchdog_error();
    const int action = controller.watchdog_action;
    const bool first_error_loop = !wd_error_seen;
    wd_error_seen = true;
    if (first_error_loop && !wd_fallback_active && only_wd &&
        (action == WD_ACTION_HOLD || action == WD_ACTION_DESCEND) &&
        watchdog_fallback_mode_ok(controller.Controller_mode))
    {
      // First loop of the trip; Controller_mode still holds the mode the host left the drive in.
      wd_fallback_active = true;
      wd_fallback_loops = 0;
      wd_hold_position = controller.Position_Ticks;
      // Hold keeps the velocity integral of a position or velocity loop (it carries the load);
      // descend starts from zero so gravity can take the joint down to the cap speed.
      if (action == WD_ACTION_DESCEND || !(controller.Controller_mode == 1 || controller.Controller_mode == 2))
        PID.V_errSum = 0;
    }
    controller.Controller_mode = 0;

    const uint32_t limit_loops = (uint32_t)controller.wd_fallback_time_ms * LOOP_FREQ / 1000;
    if (wd_fallback_active && wd_fallback_loops < limit_loops)
      wd_fallback_loops++;
    const bool expired = wd_fallback_loops >= limit_loops;

    switch (watchdog_response(action, only_wd, wd_fallback_active, expired, controller.wd_after_action))
    {
    case WD_RESP_HOLD:
      Driver_wake();
      PID.Position_setpoint = wd_hold_position;
      PID.Feedforward_speed = 0;
      PID.Feedforward_current = 0;
      Position_mode();
      break;
    case WD_RESP_DESCEND:
      Driver_wake();
      Watchdog_descend();
      break;
    case WD_RESP_BRAKE:
      Phases_brake();
      break;
    default:
      Phases_coast();
      break;
    }
  }

  /* Capture and histogram (scope.cpp), after the loop so a sample lines up with the setpoint it produced */
  Scope_tick();
  Sense_zero_late_step(); // only after a start without the motor supply

  int c2 = micros();
  controller.execution_time = c2 - c;
}

/// @brief Apply a static DC field vector at a fixed electrical angle to lock the rotor.
/// Pure +Ud (Uq = 0) pulls the rotor d-axis to 'angle'. Position_Raw is assumed already
/// refreshed by the caller's Collect_data() this tick.
static void Align_apply_field(float angle, int voltage_mV)
{
  dq0_abc_variables(angle); // sets FOC.sine_value / cosine_value / const1..4 used by abc_fast
  PID.Ud_setpoint = voltage_mV;
  PID.Uq_setpoint = 0;
  Voltage_Torque_mode();
}

/// @brief theta_offset that makes Electric_Angle == lock_angle at the current encoder position.
/// Uses the same direction-independent base angle as Collect_data(), so it is correct for
/// commutation_dir = +1 and -1 alike.
static float Align_read_offset(float lock_angle)
{
  int32_t bt = ((int32_t)controller.Position_Raw * controller.pole_pairs) % CPR;
  bt = (bt + CPR) % CPR;
  float theta_base = RAD_CONST * bt;
  float off = lock_angle - theta_base;
  while (off < 0)
    off += PI2;
  while (off >= PI2)
    off -= PI2;
  return off;
}

/// @brief Non-blocking rotor-alignment angle-offset calibration (Method A).
/// Locks the rotor to a known electrical angle with a DC field (no spinning), reads the
/// encoder, and derives theta_offset directly. Approaches the lock from both sides and
/// averages to cancel static friction/cogging bias, then verifies that +Iq drives the
/// encoder forward (the +Iq -> +encoder invariant). Direction-agnostic; unlike the
/// velocity-symmetry search it has a single stable solution per electrical cycle.
/// @return 0 in progress, 1 done (theta_offset set), -1 failed
int Calibrate_Angle_Offset_Align()
{
  enum { IN_PROGRESS = 0, DONE = 1, FAILED = -1 };

  // --- config ---
  const float LOCK_ANGLE = 0.0f;       // electrical angle we lock the rotor to
  const float DELTA = 0.6f;            // approach offset (elec rad) for both-sides averaging
  const int approach_cycles = 4000;    // ~0.64 s to drag the rotor to the approach angle
  const int settle_cycles = 8000;      // ~1.28 s to settle on the lock angle
  const int verify_settle = 6000;      // settle before measuring the verify velocity
  const int verify_measure = 300;      // velocity averaging window
  const float fwd_threshold = 1000.0f; // min |velocity| (ticks/s) to count as "spinning"
  const int MAX_CYCLES = 500000;       // ~80 s hard timeout for the lock+verify stages

  // symmetry refine (after the lock): DETERMINISTIC sweep of theta_offset around the lock seed.
  // Sample the fwd/rev speed gap at a fixed grid of offsets and keep the minimum. No gradient walk,
  // so it cannot thrash on measurement noise or land differently each run -- same result every time,
  // for commutation_dir = +-1 alike. Coarse pass finds the region; fine pass refines the coarse best.
  int trim_current = controller.calibration_offset_current; // steady-state, proven drive level
  const int trim_settle = 6000;        // ~1 s: speed steady (incl. after reversal) before measuring
  const int trim_measure = 300;        // velocity averaging window
  const float coarse_half = 0.45f;     // coarse sweep spans seed +- 0.45 rad
  const float coarse_step = 0.09f;     // -> 11 coarse points (index 0..10)
  const int coarse_points = 10;        // last coarse index
  const float fine_half = 0.09f;       // fine sweep spans coarse-best +- 0.09 rad
  const float fine_step = 0.02f;       // -> 10 fine points, ~0.02 rad final resolution
  const int fine_points = 9;           // last fine index

  static int st = 0;
  static int cnt = 0;
  static int cyc = 0;
  static int retry = 0;
  static float off_a = 0.0f;
  static float off_b = 0.0f;
  static float vel_accum = 0.0f;
  static int lock_v = 0;

  // sweep state
  static float sweep_center = 0.0f;
  static int sweep_index = 0;
  static int sweep_stage = 0; // 0 = coarse, 1 = fine
  static float best_off = 0.0f;
  static float best_gap = 0.0f;
  static float best_vfwd = 0.0f;
  static float best_vrev = 0.0f;
  static float vfwd_t = 0.0f;

  // After a calibration that was ended from outside (an E-stop) the alignment starts from its first stage. Its
  // state is kept in the statics above, and only its own done and failed exits reset them: the next #Cal used to
  // resume where the last one was cut off, and from the sweep it could commit the old result as a success.
  if (controller.Align_restart)
  {
    controller.Align_restart = 0;
    st = cnt = cyc = retry = 0;
    sweep_index = sweep_stage = 0;
    off_a = off_b = vel_accum = 0.0f;
    lock_v = 0;
  }

  cyc++;
  if (cyc > MAX_CYCLES && st < 8) // timeout guards only the lock+verify stages; the trim is iteration-bounded
  {
    PID.Ud_setpoint = 0;
    PID.Uq_setpoint = 0;
    PID.Id_setpoint = 0;
    PID.Iq_setpoint = 0;
    st = cnt = cyc = retry = 0;
    off_a = off_b = vel_accum = 0.0f;
    lock_v = 0;
    controller.Align_stage = 0;
    return FAILED;
  }

  // Progress telemetry (see common.h): st (0-12) as the stage number, sweep_index as the
  // point within the coarse/fine sweep. Printed periodically by loop(), see main.cpp.
  controller.Align_stage = st + 1;
  controller.Align_point = sweep_index;

  switch (st)
  {
  case 0: // init: derive lock voltage from measured resistance, clamp to a safe band
    lock_v = (int)((float)controller.calibration_offset_current * controller.Total_Resistance); // mA * ohm = mV
    if (lock_v < 500)
      lock_v = 500;
    if (lock_v > 4000)
      lock_v = 4000;
    cnt = 0;
    st = 1;
    break;

  case 1: // approach the lock angle from below
    Align_apply_field(LOCK_ANGLE - DELTA + PI2, lock_v);
    if (++cnt >= approach_cycles)
    {
      cnt = 0;
      st = 2;
    }
    break;

  case 2: // settle on the lock angle, then record offset A
    Align_apply_field(LOCK_ANGLE, lock_v);
    if (++cnt >= settle_cycles)
    {
      off_a = Align_read_offset(LOCK_ANGLE);
      cnt = 0;
      st = 3;
    }
    break;

  case 3: // approach the lock angle from above
    Align_apply_field(LOCK_ANGLE + DELTA, lock_v);
    if (++cnt >= approach_cycles)
    {
      cnt = 0;
      st = 4;
    }
    break;

  case 4: // settle on the lock angle, then record offset B
    Align_apply_field(LOCK_ANGLE, lock_v);
    if (++cnt >= settle_cycles)
    {
      off_b = Align_read_offset(LOCK_ANGLE);
      cnt = 0;
      st = 5;
    }
    break;

  case 5: // circular-average the two offsets and commit
  {
    float d = off_b - off_a;
    while (d > PI)
      d -= PI2;
    while (d < -PI)
      d += PI2;
    float off = off_a + d * 0.5f;
    if (off < 0)
      off += PI2;
    if (off >= PI2)
      off -= PI2;
    controller.theta_offset = off;
    controller.aligned_angle = off; // telemetry
    PID.Ud_setpoint = 0;
    PID.Uq_setpoint = 0;
    vel_accum = 0.0f;
    cnt = 0;
    st = 6;
    break;
  }

  case 6: // verify: apply +Iq and let it settle
    PID.Id_setpoint = 0;
    PID.Iq_current_limit = 1600;
    PID.Iq_setpoint = controller.calibration_offset_current;
    Torque_mode();
    if (++cnt >= verify_settle)
    {
      cnt = 0;
      st = 7;
    }
    break;

  case 7: // verify: average velocity under +Iq, enforce +Iq -> +encoder
    PID.Iq_setpoint = controller.calibration_offset_current;
    Torque_mode();
    vel_accum += controller.Velocity_Average;
    if (++cnt >= verify_measure)
    {
      float vfwd = vel_accum / verify_measure;
      controller.Velocity_fwd = (int)vfwd; // telemetry
      PID.Iq_setpoint = 0;
      PID.Id_setpoint = 0;

      if (vfwd > fwd_threshold)
      {
        // +Iq -> +encoder confirmed; refine with the measured-direction symmetry search (keep-best)
        cnt = 0;
        st = 8;
      }
      else if (vfwd < -fwd_threshold && retry == 0)
      {
        // +Iq -> -encoder: 180-deg reference, flip once and re-verify
        controller.theta_offset += PI;
        if (controller.theta_offset >= PI2)
          controller.theta_offset -= PI2;
        controller.aligned_angle = controller.theta_offset;
        retry = 1;
        vel_accum = 0.0f;
        cnt = 0;
        st = 6;
      }
      else
      {
        // still reversed after a flip, or not spinning at all -> fail
        st = cnt = cyc = retry = 0;
        off_a = off_b = vel_accum = 0.0f;
        lock_v = 0;
        controller.Align_stage = 0;
        return FAILED;
      }
    }
    break;

  case 8: // sweep init: center the grid on the lock's offset, start the coarse pass
    PID.Iq_current_limit = 1600;
    sweep_center = controller.theta_offset;
    sweep_stage = 0;
    sweep_index = 0;
    best_gap = 1e12f;
    best_off = controller.theta_offset;
    best_vfwd = 0.0f;
    best_vrev = 0.0f;
    controller.theta_offset = sweep_center - coarse_half; // first coarse point
    if (controller.theta_offset < 0)
      controller.theta_offset += PI2;
    vel_accum = 0.0f;
    cnt = 0;
    st = 9;
    break;

  case 9: // sweep: settle under +Iq at the current grid offset
    PID.Id_setpoint = 0;
    PID.Iq_setpoint = trim_current;
    Torque_mode();
    if (++cnt >= trim_settle)
    {
      cnt = 0;
      vel_accum = 0.0f;
      st = 10;
    }
    break;

  case 10: // sweep: measure forward speed
    PID.Iq_setpoint = trim_current;
    Torque_mode();
    vel_accum += controller.Velocity_Average;
    if (++cnt >= trim_measure)
    {
      vfwd_t = vel_accum / trim_measure;
      cnt = 0;
      st = 11;
    }
    break;

  case 11: // sweep: settle under -Iq
    PID.Iq_setpoint = -trim_current;
    Torque_mode();
    if (++cnt >= trim_settle)
    {
      cnt = 0;
      vel_accum = 0.0f;
      st = 12;
    }
    break;

  case 12: // sweep: measure reverse speed, record the gap, advance the grid
  {
    PID.Iq_setpoint = -trim_current;
    Torque_mode();
    vel_accum += controller.Velocity_Average;
    if (++cnt >= trim_measure)
    {
      float vrev_t = vel_accum / trim_measure;
      float gap = fabs(vfwd_t + vrev_t); // 0 when |fwd| == |rev|
      cnt = 0;

      if (gap < best_gap)
      {
        best_gap = gap;
        best_off = controller.theta_offset;
        best_vfwd = vfwd_t;
        best_vrev = vrev_t;
      }

      sweep_index++;

      if (sweep_stage == 0)
      {
        if (sweep_index <= coarse_points)
        {
          controller.theta_offset = sweep_center - coarse_half + sweep_index * coarse_step;
        }
        else
        {
          // coarse pass done -> fine pass around the coarse best
          sweep_stage = 1;
          sweep_index = 0;
          sweep_center = best_off;
          controller.theta_offset = sweep_center - fine_half;
        }
      }
      else // fine pass
      {
        if (sweep_index <= fine_points)
        {
          controller.theta_offset = sweep_center - fine_half + sweep_index * fine_step;
        }
        else
        {
          // done: commit the best offset found (deterministic, same every run)
          controller.theta_offset = best_off;
          controller.aligned_angle = best_off;
          controller.Velocity_fwd = (int)best_vfwd; // telemetry: symmetry at the committed offset
          controller.Velocity_bwd = (int)best_vrev;
          PID.Iq_setpoint = 0;
          PID.Id_setpoint = 0;
          st = cnt = cyc = retry = 0;
          off_a = off_b = vel_accum = 0.0f;
          lock_v = 0;
          sweep_index = 0;
          sweep_stage = 0;
          controller.Align_stage = 0;
          return DONE;
        }
      }

      if (controller.theta_offset < 0)
        controller.theta_offset += PI2;
      if (controller.theta_offset >= PI2)
        controller.theta_offset -= PI2;

      st = 9; // measure the next grid point
    }
    break;
  }
  }

  return IN_PROGRESS;
}

/// @brief Interrupt callback routine for Calibration mode
void Update_IT_callback_calib()
{
  IWatchdog.reload();
  // The driver awake for the whole calibration: #Cal wakes it, but an idle loop of IT_callback() could still run
  // before this interrupt took over and put it back to sleep ("Resistance test: Failed" at once).
  Driver_wake();
  // An E-stop ends the calibration. Nothing looked at it here: the motor kept turning (the KV spins run at 1250 mA)
  // until the calibration was through.
  if (controller.ESTOP_error && controller.Calib_error == 0)
  {
    Phases_coast();
    controller.Calib_error = 1;
  }

  /// Flags
  static int calib_step_magnet = 0;
  static int calib_step_voltage = 0;
  static int calib_step_resistance = 0;
  static int calib_step_inductance = 0;
  static int calib_step_delay = 0;
  static int calib_dir_pole_pair = 0;
  static int Phase_order_step = 0;
  static int KV_step = 0;

  static int tick_cnt = 0;
  static float resistance_voltage = 0;

  static int check_Vbus = 0;
  static int check_magnet = 0;

  volatile float EMA_a_R = 0.05; //  0.05 with 2V
  static float Res_avg = 0;
  static int Resistance_accumulator = 0;

  static int delay = 200; // Delay between resistance and inductance mesurement

  static int Inductor_start = 0;
  static float inductance_accumulator = 0.0;
  static int inductance_step = 0;
  static int inductance_sample_size = 100000;
  static int inductance_tick = 0;
  static int inductance_tick_2 = 0;

  static int Phase_step = 0;
  static int Phase_ticks = 0;
  static int Phase_start = 0;
  static int64_t _speed_accumulator = 0;
  static int Spin_duration = 8500; // In interrupt ticks = 8500 * 200us = 1.7s

  // Spin-confirm probe angle bias: a pure-Uq command uses the still-uncalibrated
  // encoder-derived angle directly, so how much of it lands on the true torque axis
  // depends on this motor's actual (unknown) magnet-to-encoder mounting angle. Two
  // trials 90 electrical degrees apart guarantee at least one lands within 45 degrees
  // of the true q-axis (>=1/sqrt(2) ~= 70.7% of full command magnitude, for ANY true
  // offset -- max(|cos|,|sin|) >= 1/sqrt(2) always), comfortably enough to move a motor
  // that would otherwise pass this test outright.
  static int probe_attempt = 0;

  static int KV_ticks = 0;
  static int KV_duration = 5000;
  static int64_t KV_speed_accumulator = 0;
  static int64_t KV_current_accumulator = 0;
  static int KV_run = 0;  // which of the KV_RUNS spins
  static int KV_wait = 0; // ticks left braking between two spins

  // Open loop
  static float theta_SIM = 0;
  static int velocity_step = 0;
  static int open_loop_start = 0;
  static int start_position = 0;
  static int move_stop = 0;
  static int tick_tick = 0;
  static int open_loop_delay = 0;
  static float open_loop_step = 0.087266;

  /////////////////////////////////////////////////////////////////////
  /*
Check if there is magnet
*/
  if (calib_step_magnet == 0 && controller.Calib_error == 0)
  {
    controller.Calibration = 1;
    Collect_data();
    // If there is error
    if (controller.Magnet_warrning == 1)
    {
      controller.Magnet_cal_status = 1;
      calib_step_magnet = 0;
      controller.Calib_error = 1;
    }
    else // If there is no error
    {
      controller.Magnet_cal_status = 2;
      calib_step_magnet = 1;
    }
  }

  /////////////////////////////////////////////////////////////////////
  /*
Check if the Vbus is in the range
*/
  if (calib_step_voltage == 0 && calib_step_magnet == 1 && controller.Calib_error == 0)
  {
    controller.VBUS_RAW = ADC_CHANNEL_6_READ_VBUS();
    controller.VBUS_mV = Get_voltage_mV(controller.VBUS_RAW);
    // If there is error
    if (controller.VBUS_mV < controller.Min_Vbus || controller.VBUS_mV > controller.Max_Vbus)
    {
      controller.Vbus_cal_status = 1;
      calib_step_voltage = 0;
      controller.Calib_error = 1;
    }
    else // If there is no error
    {
      controller.Vbus_cal_status = 2;
      calib_step_voltage = 1;
      resistance_voltage = controller.Resistance_calc_voltage;
    }
  }

  /////////////////////////////////////////////////////////////////////
  /*
Calculate resistance of BLDC phase
*/
  if (calib_step_resistance == 0 && calib_step_voltage == 1 && controller.Calib_error == 0)
  {

    // Disable one branch of the inverter
    controller.VBUS_RAW = ADC_CHANNEL_6_READ_VBUS();
    controller.VBUS_mV = Get_voltage_mV(controller.VBUS_RAW);
    digitalWriteFast(EN1, HIGH);
    digitalWriteFast(EN2, LOW);
    digitalWriteFast(EN3, HIGH);
    FOC.PWM1 = 0;
    FOC.PWM2 = 0;
    FOC.PWM3 = (int)(resistance_voltage * (float)((float)PWM_MAX / (float)controller.VBUS_mV));
    pwm_set(PWM_CH1, 0, 13);
    pwm_set(PWM_CH2, 0, 13);
    pwm_set(PWM_CH3, FOC.PWM3, 13);
    tick_cnt = tick_cnt + 1;
    if (tick_cnt >= 150) // 150
    {
      Sense_sample_wait();
      controller.Sense1_Raw = ADC_CHANNEL_4_READ_SENSE1() - controller.sense1_offset;
      controller.Sense2_Raw = ADC_CHANNEL_3_READ_SENSE2() - controller.sense2_offset;
      controller.Sense1_mA = Get_current_mA(controller.Sense1_Raw);
      controller.Sense2_mA = Get_current_mA(controller.Sense2_Raw);
      int cur_avg = (abs(controller.Sense1_mA) + abs(controller.Sense2_mA)) / 2; // phase C in, phase A out
      // controller.Resistance = (( (float)controller.VBUS_mV * ((float)FOC.PWM3 / (float) PWM_MAX)) / (float) controller.Sense1_Raw);
      controller.Resistance = (float)(resistance_voltage - (float)((2 * SENSE_RESISTOR + Rdson * 2) * cur_avg)) / (float)(2 * cur_avg);
      Res_avg = qfp_fadd(qfp_fmul(EMA_a_R, controller.Resistance), qfp_fmul(qfp_fsub(1, EMA_a_R), Res_avg)); // Units [A]
      controller.Resistance = Res_avg;

      if (tick_cnt >= 500) // 18000
      {
        // Check if motor is connected, resistance cant be too large, negative or 0
        if (controller.Resistance > 1000 || controller.Resistance <= 0.01)
        {
          controller.Resistance_cal_status = 1;
          controller.Total_Resistance = 0;
          controller.Resistance = 0;
          controller.Calib_error = 1;
        }
        else // If there is normal resistance
        {

          if ((float)((float)resistance_voltage / (float)1000) * (float)((float)cur_avg / (float)1000) < (controller.Calibration_power))
          {
            resistance_voltage = resistance_voltage + 200;
            tick_cnt = 0;
          }
          else
          {
            controller.Total_Resistance = controller.Resistance * 2 + 2 * SENSE_RESISTOR + Rdson * 2;
            // No "* 1.25" any more (en2fix, 2026-11-10): it made up for phase B conducting in parallel, back when EN2
            // was the clock output and "EN2 LOW" did nothing. With phase B really off, this is the phase resistance.
            controller.Resistance_cal_status = 2;
            calib_step_resistance = 1;
          }
        }

        tick_cnt = 0;
        FOC.PWM1 = 0;
        FOC.PWM2 = 0;
        FOC.PWM3 = 0;
        pwm_set(PWM_CH1, 0, 13);
        pwm_set(PWM_CH2, 0, 13);
        pwm_set(PWM_CH3, 0, 13);
        digitalWriteFast(EN1, HIGH);
        digitalWriteFast(EN2, LOW);
        digitalWriteFast(EN3, HIGH);
      }
    }
  }

  /////////////////////////////////////////////////////////////////////
  /*
Small delay between resistance mesure and inductance mesure to drain the coil
*/
  if (calib_step_delay == 0 && calib_step_resistance == 1 && controller.Calib_error == 0)
  {
    tick_cnt = tick_cnt + 1;
    if (tick_cnt == delay)
    {
      calib_step_delay = 1;
    }
  }

  /////////////////////////////////////////////////////////////////////
  // Get inductance, Ld=Lq=0.5*Ls(line to line) This applies for PMSM BLDC motor
  // We mesure line to line inductance and devide it by 2
  if (calib_step_delay == 1 && calib_step_inductance == 0 && controller.Calib_error == 0)
  {

    if (inductance_tick_2 == 0)
    {
      // Setup what voltage to use based on desired Power
      if (Inductor_start == 0)
      {
        FOC.PWM1 = 0;
        FOC.PWM2 = 0;
        controller.Calibration_Inductance_voltage = (int)(sqrt(controller.Calibration_power * controller.Total_Resistance) * 1000);
        if (controller.Calibration_Inductance_voltage > controller.VBUS_mV)
        {
          controller.Calibration_Inductance_voltage = controller.VBUS_mV;
        }
        FOC.PWM3 = (int)(controller.Calibration_Inductance_voltage * (float)((float)PWM_MAX / (float)controller.VBUS_mV));
        pwm_set(PWM_CH1, 0, 13);
        pwm_set(PWM_CH2, 0, 13);
        pwm_set(PWM_CH3, FOC.PWM3, 13);
        Inductor_start = 1;
      }

      else
      {

        controller.VBUS_RAW = ADC_CHANNEL_6_READ_VBUS();
        controller.VBUS_mV = Get_voltage_mV(controller.VBUS_RAW);
        Sense_sample_wait();
        controller.Sense1_Raw = ADC_CHANNEL_4_READ_SENSE1() - controller.sense1_offset;
        controller.Sense2_Raw = ADC_CHANNEL_3_READ_SENSE2() - controller.sense2_offset;
        controller.Sense1_mA = Get_current_mA(controller.Sense1_Raw);
        controller.Sense2_mA = Get_current_mA(controller.Sense2_Raw);
        int cur_avg = (abs(controller.Sense1_mA) + abs(controller.Sense2_mA)) / 2; // phase C in, phase A out
        int V_inductor = controller.Calibration_Inductance_voltage - controller.Total_Resistance * cur_avg;
        controller.Inductance = (float)((float)V_inductor / (float)1000) / (((float)cur_avg / (float)1000) / (float)LOOP_TIME);
        inductance_step = inductance_step + 1;
        inductance_accumulator = controller.Inductance + inductance_accumulator;
        inductance_tick_2 = 1;
        pwm_set(PWM_CH1, 0, 13);
        pwm_set(PWM_CH2, 0, 13);
        pwm_set(PWM_CH3, 0, 13);
      }
    }
    else
    {
      inductance_tick = inductance_tick + 1;
      if (inductance_tick == delay)
      {
        inductance_tick_2 = 0;
        inductance_tick = 0;
        Inductor_start = 0;
      }
    }

    if (inductance_step == 100)
    {

      controller.Inductance = (float)(inductance_accumulator) / (float)100 / (float)2;
      // If there is error
      if (isnan(controller.Inductance))
      {
        controller.Inductance = 0;
        controller.Inductance_cal_status = 1;
        calib_step_inductance = 0;
        controller.Calib_error = 1;
      }
      else // If there is no error
      {
        // Will set PI gains for Q and D current loops based on phase resistance
        // and phase inductance with -3dB bandwidth at desired frequency
        controller.Inductance_cal_status = 2;
        calib_step_inductance = 1;
        if (!Current_gains_from_RL()) // Kp = w L, Ki = w R at current_control_bandwidth
        {
          // zero, negative or not a number: the calibration fails. It used to stay in this step, pulsing for ever.
          controller.Inductance = 0;
          controller.Inductance_cal_status = 1;
          controller.Calib_error = 1;
          calib_step_inductance = 0;
        }
      }
      digitalWriteFast(EN1, HIGH);
      digitalWriteFast(EN2, HIGH);
      digitalWriteFast(EN3, HIGH);
    }
  }

  /////////////////////////////////////////////////////////////////////
  /*
Open loop spin; Get pole pair and dir
*/
  if (calib_step_inductance == 1 && calib_dir_pole_pair == 0 && controller.Calib_error == 0)
  {
    tick_tick = tick_tick + 1;
    // Set initial position and speed
    if (open_loop_start == 0)
    {
      // Simulate speed for open loop operation
      velocity_step = (int)((float)open_loop_step / ((float)(controller.Open_loop_speed) / (float)CPR));
      theta_SIM = 0;
      theta_SIM = wrap_2pi(theta_SIM);
      if (theta_SIM < 0)
      {
        theta_SIM += PI2;
      }

      Collect_data();
      dq0_abc_variables(theta_SIM);

      abc_fast(0, controller.Open_loop_voltage, &FOC.U1, &FOC.U2, &FOC.U3);
      sinusoidal_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);
      // space_vector_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);

      FOC.PWM1 = FOC.U1_normalized * PWM_MAX;
      FOC.PWM2 = FOC.U2_normalized * PWM_MAX;
      FOC.PWM3 = FOC.U3_normalized * PWM_MAX;

      pwm_set(PWM_CH1, FOC.PWM1, 13);
      pwm_set(PWM_CH2, FOC.PWM2, 13);
      pwm_set(PWM_CH3, FOC.PWM3, 13);

      open_loop_start = 1;
      // Detect the wiring from the UN-swapped state; a board already saved as reversed must not
      // spin with the software B<->C swap active, or it would mis-detect its own wiring.
      controller.commutation_dir = 1;
    }

    // Read the position here after it becomes stable; we introduce small delay = 20000 * LOOP_TIME
    if (tick_tick == 20000 && open_loop_delay == 0 && open_loop_start == 1)
    {
      Collect_data();
      start_position = controller.Position_Ticks;
      open_loop_delay = 1;
      tick_tick = 0;
    }

    // Spin the motor Open loop for "number_of_elec_rotations"
    if (tick_tick == velocity_step && open_loop_delay == 1 && move_stop == 0 && theta_SIM < PI2 * controller.Number_of_rotations)
    {
      tick_tick = 0;
      theta_SIM = (float)theta_SIM + (float)open_loop_step; // velocity_SIM * LOOP_TIME;
      float theta_SIM_command;
      if (theta_SIM > PI2 * controller.Number_of_rotations)
      {

        move_stop = 1;
      }

      theta_SIM_command = wrap_2pi(theta_SIM);
      if (theta_SIM_command < 0)
      {
        theta_SIM_command += PI2;
      }

      Collect_data();
      dq0_abc_variables(theta_SIM_command);

      abc_fast(0, controller.Open_loop_voltage, &FOC.U1, &FOC.U2, &FOC.U3);
      sinusoidal_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);
      // space_vector_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);

      FOC.PWM1 = FOC.U1_normalized * PWM_MAX;
      FOC.PWM2 = FOC.U2_normalized * PWM_MAX;
      FOC.PWM3 = FOC.U3_normalized * PWM_MAX;

      pwm_set(PWM_CH1, FOC.PWM1, 13);
      pwm_set(PWM_CH2, FOC.PWM2, 13);
      pwm_set(PWM_CH3, FOC.PWM3, 13);
    }
    // After we spun the motor
    if (move_stop == 1)
    {
      // Number_of_rotations electrical turns move the rotor that many pole pitches. In float: the integer division
      // that was here gave a division by zero (reported as 0 pole pairs) when the rotor moved under 8 ticks.
      const int moved_ticks = abs(controller.Position_Ticks - start_position);
      controller.pole_pairs = (moved_ticks > 0) ? (int)round((float)CPR * (float)controller.Number_of_rotations / (float)moved_ticks) : 0;
      // If there is error. Pole pair cant be 0 or negative and we limit upper limit to 100PP
      // 1 to 49: 50 and more is what the product check takes for a STEPFOC calibration (EEPROM.cpp), so a saved
      // result above it made the board report "wrong product" at the next start.
      if (controller.pole_pairs <= 0 || controller.pole_pairs > 49)
      {
        controller.Open_loop_cal_status = 1;
        controller.pole_pairs = 0;
        controller.Calib_error = 1;
        calib_dir_pole_pair = 0;
      }
      else // If there is no error
      {
        // Detect the wiring HANDEDNESS from the encoder response to the forward open-loop command,
        // instead of failing / picking a DIR_ that only some wirings can satisfy. The open loop
        // commanded an increasing electrical angle, so:
        //   encoder advanced -> +Iq maps to +encoder in the base frame -> commutation_dir = +1
        //   encoder went back -> phases are reversed -> commutation_dir = -1 (B<->C reflection on)
        controller.Open_loop_cal_status = 2;
        controller.commutation_dir = (controller.Position_Ticks >= start_position) ? 1 : -1;
        // Fixed base frame: the rotation (which cyclic wiring) is absorbed by theta_offset in
        // Calibrate_Angle_Offset_Align, and the handedness by commutation_dir, so we no longer
        // search DIR_/Phase_order -- pin them to the base mapping.
        controller.DIR_ = 1;
        controller.Phase_order = 2;
        calib_dir_pole_pair = 1;
      }
    }
  }

  /////////////////////////////////////////////////////////////////////
  /*
Confirm commutation, then find the angle offset (reversed-wiring-agnostic)

  Was: spin 3 cyclic PWM orderings and require the fastest to be order 2, else fail with
  "switch motor phases". Now the base frame is fixed (DIR_=1, Phase_order=2) and the wiring is
  compensated entirely in software: commutation_dir (detected in the open-loop spin) handles the
  handedness via the B<->C reflection, and theta_offset (Calibrate_Angle_Offset_Align) handles the
  rotation over the full electrical cycle. So all we do here is (a) confirm the motor actually
  commutates in EITHER direction, then (b) run the deterministic rotor-lock angle-offset search.
*/
  if (calib_dir_pole_pair == 1 && Phase_order_step == 0 && controller.Calib_error == 0)
  {

    if (Phase_start == 0)
    {
      controller.VBUS_RAW = ADC_CHANNEL_6_READ_VBUS();
      controller.VBUS_mV = Get_voltage_mV(controller.VBUS_RAW);
      // Check if vbus is too small for setpoint voltage
      // If it is throw error
      if (controller.VBUS_mV < controller.Phase_voltage)
      {
        Phase_start = 0;
        controller.Kt_cal_status = 1;
        controller.Calib_error = 1;
      }
      else
      {
        Phase_start = 1;
      }
    }
    else
    {
      Phase_ticks = Phase_ticks + 1;
      // Which angle bias this attempt probes with -- see probe_attempt's declaration above.
      float probe_bias = (probe_attempt == 0) ? 0.0f : (PI / 2.0f);
      switch (Phase_step)
      {
      case 0: // Small delay, hold the rotor at zero torque before the spin test
        if (Phase_ticks == 10000)
        {
          Phase_step = 1;
          Phase_ticks = 0;
        }
        else
        {
          Collect_data2();
          dq0_abc_variables(controller.Electric_Angle + probe_bias);
          dq0_fast_int(controller.Sense1_mA, controller.Sense2_mA, controller.Sense3_mA, &FOC.Id, &FOC.Iq);
          PID.Ud_setpoint = 0;
          PID.Uq_setpoint = 0;
          Voltage_Torque_mode();
        }
        break;

      case 1: // Record the initial position
        Collect_data2();
        start_position = controller.Position_Ticks;
        Phase_step = 2;
        break;

      case 2: // Spin with a voltage-torque command and confirm the motor actually turned
        if (Phase_ticks < Spin_duration)
        {
          Collect_data2();
          dq0_abc_variables(controller.Electric_Angle + probe_bias);
          dq0_fast_int(controller.Sense1_mA, controller.Sense2_mA, controller.Sense3_mA, &FOC.Id, &FOC.Iq);
          PID.Ud_setpoint = 0;
          PID.Uq_setpoint = controller.Phase_voltage;
          Voltage_Torque_mode();
        }
        else
        {
          int delta = controller.Position_Ticks - start_position;
          int half_rotation_ticks = CPR / 2;
          // Direction-agnostic: only confirm the motor moved (commutation works). The correct
          // sense (+Iq -> +encoder) is enforced by the lock's verify step, so we must NOT fail /
          // tell the user to switch phases here.
          if (abs(delta) > half_rotation_ticks)
          {
            probe_attempt = 0; // reset for the next #Cal run
            Phase_step = 3;    // moved far enough in EITHER direction
          }
          else
          {
            // Diagnostic: stash how far it actually got for Calib_report() (main-loop context)
            // to print later -- printing directly from here (the calibration ISR) races with
            // Calib_report()'s own prints on the same UART and tears the output.
            controller.Spin_confirm_delta = delta;
            controller.Spin_confirm_vbus_mV = controller.VBUS_mV;
            PID.Uq_setpoint = 0;
            Voltage_Torque_mode();
            Phase_ticks = 0;
            Phase_start = 1;
            Phase_step = 0;

            if (probe_attempt == 0)
            {
              // This bias produced ~no motion -- try again 90 degrees away before giving up.
              probe_attempt = 1;
            }
            else
            {
              // Both biases tried, neither moved the motor -> commutation genuinely not working.
              probe_attempt = 0; // reset for the next #Cal run
              controller.Kt_cal_status = 1;
              controller.Phase_order_status = 1;
              controller.Calib_error = 1;
            }
          }
        }
        break;

      case 3: // Seed the angle-offset search
        controller.theta_offset = 0;
        Phase_step = 4;
        break;

      case 4: // Deterministic rotor-lock + verify + symmetry-sweep angle-offset calibration
        Collect_data();
        dq0_abc_variables(controller.Electric_Angle);
        dq0_fast_int(controller.Sense1_mA, controller.Sense2_mA, controller.Sense3_mA, &FOC.Id, &FOC.Iq);
        controller.temp_var_offset_calib = Calibrate_Angle_Offset_Align();
        if (controller.temp_var_offset_calib == 1)
        {
          Phase_step = 5; // done
        }
        else if (controller.temp_var_offset_calib == -1)
        {
          // clean abort: alignment could not satisfy +Iq -> +encoder
          Phase_ticks = 0;
          Phase_start = 1;
          Phase_step = 0;
          controller.Kt_cal_status = 1;
          controller.Phase_order_status = 1;
          controller.Calib_error = 1;
        }
        break;

      case 5: // End: previous steps were good, go to the Kt/KV calculation
        pwm_set(PWM_CH1, 0, 13);
        pwm_set(PWM_CH2, 0, 13);
        pwm_set(PWM_CH3, 0, 13);
        Phase_ticks = 0;
        Phase_start = 1;
        Phase_step = 0;
        controller.Phase_order = 2; // fixed base frame
        Phase_order_step = 1;
        controller.Kt_cal_status = 2;
        controller.Phase_order_status = 2;
        controller.Calib_error = 0;
        break;
      }
    }
  }

  /// Calculate Kt, KV and Flux linkage: KV_RUNS spins from standstill, alternately forward and back, braking between
  /// them, each divided by the voltage actually available in exact volts. KV is their mean; the log shows how far they
  /// agree.
  if (Phase_order_step == 1 && KV_step == 0 && controller.Calib_error == 0)
  {
    if (KV_wait > 0)
    {
      KV_wait = KV_wait - 1;
      Collect_data(); // the speed's moving average runs down to 0 before the next spin
    }
    else if (KV_ticks <= KV_duration)
    {
      if (KV_ticks == 0)
      {
        PID.Iq_errSum = 0.0f; // nothing left from the spin before
        PID.Id_errSum = 0.0f;
      }
      PID.Iq_current_limit = 1600;
      PID.Iq_setpoint = (KV_run & 1) ? -1250 : 1250;
      Collect_data();
      Torque_mode();
      dq0_abc_variables(controller.Electric_Angle);
      dq0_fast_int(controller.Sense1_mA, controller.Sense2_mA, controller.Sense3_mA, &FOC.Id, &FOC.Iq);
      KV_ticks = KV_ticks + 1;
      KV_speed_accumulator = KV_speed_accumulator + controller.Velocity_Average;
      KV_current_accumulator = KV_current_accumulator + FOC.Iq;
    }
    else
    {
      float rpm_filtered = (float)KV_speed_accumulator * 60.0f / 16384.0f / (float)KV_duration;
      if (rpm_filtered < 0.0f)
        rpm_filtered = -rpm_filtered;
      const float current_filtered = (float)KV_current_accumulator / (float)KV_duration;
      const float current_gap = PID.Iq_setpoint - current_filtered;
      int load_correction = current_gap != 0.0f ? (int)(PID.Iq_setpoint / current_gap) : 1;
      if (load_correction < 1)
        load_correction = 1;
      const int spin_mV = (PID.Voltage_limit > 0 && PID.Voltage_limit < controller.VBUS_mV) ? PID.Voltage_limit : controller.VBUS_mV;
      float KV_value = rpm_filtered * 1000.0f / (float)spin_mV * (float)load_correction;
      controller.KV_runs[KV_run] = KV_value;
      KV_run = KV_run + 1;
      pwm_set(PWM_CH1, 0, 13);
      pwm_set(PWM_CH2, 0, 13);
      pwm_set(PWM_CH3, 0, 13);
      KV_ticks = 0;
      KV_speed_accumulator = 0;
      KV_current_accumulator = 0;
      if (KV_run < KV_RUNS)
      {
        KV_wait = KV_BRAKE_TICKS;
      }
      else
      {
        // Spread: runs of the same direction only. Forward and back can differ by a few percent while the electrical
        // offset is still a little off (a first calibration, no encoder table yet); their mean is right regardless.
        float sum = 0.0f, worst = 0.0f;
        for (int k = 0; k < KV_RUNS; k++)
        {
          sum += controller.KV_runs[k];
          if (k >= 2)
          {
            float gap = controller.KV_runs[k] - controller.KV_runs[k - 2];
            if (gap < 0.0f)
              gap = -gap;
            if (gap > worst)
              worst = gap;
          }
        }
        controller.KV = sum / (float)KV_RUNS;
        controller.KV_spread_pct = controller.KV > 0.0f ? worst / controller.KV * 100.0f : 0.0f;
        controller.Kt = float(8.27) / controller.KV;
        controller.flux_linkage = (float)2 / 3 * (float)controller.Kt / (float)controller.pole_pairs;
        controller.temp1 = controller.KV;
        KV_step = 1;
        controller.KV_status = 2;
      }
    }
  }

  // If we get to last step without error
  if (KV_step == 1)
  {
    // Detach calibration interrupt routine
    Ticker_detach(TIM3);
    controller.Calibration = 3;
    controller.Calibrated = 1;
    // Reset all flags
    calib_step_magnet = 0;
    calib_step_voltage = 0;
    calib_step_resistance = 0;
    calib_step_inductance = 0;
    calib_step_delay = 0;
    Phase_order_step = 0;
    calib_dir_pole_pair = 0;
    tick_cnt = 0;
    check_Vbus = 0;
    check_magnet = 0;
    Res_avg = 0;
    Resistance_accumulator = 0;
    Inductor_start = 0;
    inductance_accumulator = 0.0;
    inductance_step = 0;
    inductance_tick = 0;
    inductance_tick_2 = 0;
    Phase_step = 0;
    Phase_ticks = 0;
    Phase_start = 0;
    _speed_accumulator = 0;
    theta_SIM = 0;
    velocity_step = 0;
    open_loop_start = 0;
    start_position = 0;
    move_stop = 0;
    tick_tick = 0;
    open_loop_delay = 0;
    resistance_voltage = 0;
    KV_ticks = 0;
    KV_speed_accumulator = 0;
    KV_current_accumulator = 0;
    KV_step = 0;
    KV_run = 0;
    KV_wait = 0;
  }

  // If there is error
  if (controller.Calib_error == 1)
  {
    // Detach calibration interrupt routine
    Ticker_detach(TIM3);
    Phases_coast(); // bridge off now: the last vector used to stay applied until the main loop printed its report
    // The resistance and inductance steps switch phase B off (EN2 low): all three back on, or a failed calibration
    // leaves phase B floating in normal running until a reboot (en2fix: EN2 is a real pin now).
    digitalWriteFast(EN1, HIGH);
    digitalWriteFast(EN2, HIGH);
    digitalWriteFast(EN3, HIGH);
    controller.Calibration = 2;
    controller.Calibrated = 0;
    Cal_restore_current_limit(); // batch 22d
    // the angle alignment may have been cut off half-way (an E-stop): it starts over next time, its progress line
    // stops, and nothing it commanded stays behind for a later voltage or current command
    controller.Align_restart = 1;
    controller.Align_stage = 0;
    probe_attempt = 0;
    PID.Ud_setpoint = 0;
    PID.Uq_setpoint = 0;
    PID.Id_setpoint = 0;
    PID.Iq_setpoint = 0;
    // Reset all flags
    calib_step_magnet = 0;
    calib_step_voltage = 0;
    calib_step_resistance = 0;
    calib_step_inductance = 0;
    calib_step_delay = 0;
    Phase_order_step = 0;
    calib_dir_pole_pair = 0;
    tick_cnt = 0;
    check_Vbus = 0;
    check_magnet = 0;
    Res_avg = 0;
    Resistance_accumulator = 0;
    Inductor_start = 0;
    inductance_accumulator = 0.0;
    inductance_step = 0;
    inductance_tick = 0;
    inductance_tick_2 = 0;
    Phase_step = 0;
    Phase_ticks = 0;
    Phase_start = 0;
    _speed_accumulator = 0;
    theta_SIM = 0;
    velocity_step = 0;
    open_loop_start = 0;
    start_position = 0;
    move_stop = 0;
    tick_tick = 0;
    open_loop_delay = 0;
    resistance_voltage = 0;
    KV_ticks = 0;
    KV_speed_accumulator = 0;
    KV_current_accumulator = 0;
    KV_step = 0;
    KV_run = 0;
    KV_wait = 0;
  }
}

/// @brief Spin the motor in the open loop
/// @param speed Speed in RAD/s; NOTE this is electrical speed of the motor! Speed_mech = speed_el / Pole_pairs
/// @param voltage Voltage in milivolts
void Open_loop_speed(float speed_el, int voltage)
{

  static float theta_SIM = 0;
  theta_SIM += speed_el * LOOP_TIME;
  theta_SIM = wrap_2pi(theta_SIM);
  if (theta_SIM < 0)
  {
    theta_SIM += PI2;
  }

  dq0_abc_variables(theta_SIM);
  abc_fast(voltage, 0, &FOC.U1, &FOC.U2, &FOC.U3);
  // sinusoidal_commutation(12000, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);
  space_vector_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);

  FOC.PWM1 = FOC.U1_normalized * PWM_MAX;
  FOC.PWM2 = FOC.U2_normalized * PWM_MAX;
  FOC.PWM3 = FOC.U3_normalized * PWM_MAX;

  if (controller.DIR_ == 1)
  {
    pwm_set(PWM_CH1, FOC.PWM1, 13);
    pwm_set(PWM_CH2, FOC.PWM2, 13);
    pwm_set(PWM_CH3, FOC.PWM3, 13);
  }
  else
  {
    pwm_set(PWM_CH2, FOC.PWM1, 13);
    pwm_set(PWM_CH1, FOC.PWM2, 13);
    pwm_set(PWM_CH3, FOC.PWM3, 13);
  }
}

/// @todo THIS
/// @brief Step the BLDC open loop. Operates like stepper motor
/// @param step 0 - 2pi that is transformed to Ua,Ub,Uc. 0 - 2pi is equal to ONE ELECTRICAL ROTATION
/// @param voltage Voltage in milivolts
void Open_loop_step(int step, int voltage)
{
}

/// @brief Reports data when in calibration mode
/// @param Serialport
void Calib_report(Stream &Serialport)
{

  static bool print_flag[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  if (controller.Calibration != 0)
  {
    if (controller.Calibration == 1 && print_flag[0] == 0)
    {
      Serialport.print("----------------");
      Serialport.println(" ");
      Serialport.print("Calibration mode");
      Serialport.println(" ");
      Serialport.print("-----Settings-----");
      Serialport.println(" ");
      Serialport.print("Current loop bandwidth: ");
      Serialport.print(controller.current_control_bandwidth);
      Serialport.print("Hz");
      Serialport.println(" ");
      Serialport.print("Resistance voltage: ");
      Serialport.print(controller.Resistance_calc_voltage);
      Serialport.print("mV");
      Serialport.println(" ");
      Serialport.print("Max Power dissipation: ");
      Serialport.print(controller.Calibration_power);
      Serialport.print("W");
      Serialport.println(" ");
      Serialport.print("Open loop voltage: ");
      Serialport.print(controller.Open_loop_voltage);
      Serialport.print("mV");
      Serialport.println(" ");
      Serialport.print("Open loop Speed: ");
      Serialport.print(controller.Open_loop_speed);
      Serialport.print("Rad/s");
      Serialport.println(" ");
      Serialport.print("Phase search voltage: ");
      Serialport.print(controller.Phase_voltage);
      Serialport.print("mV");
      Serialport.println(" ");
      Serialport.print("----------------");
      Serialport.println(" ");

      print_flag[0] = 1;
    }

    /// Magnet test status
    if (controller.Magnet_cal_status != 0 && print_flag[1] == 0)
    {
      if (controller.Magnet_cal_status == 1)
      {
        Serialport.print("Magnet test: Failed");
        Serialport.println(" ");
      }
      else if (controller.Magnet_cal_status == 2)
      {
        Serialport.print("Magnet test: Passed");
        Serialport.println(" ");
      }
      print_flag[1] = 1;
    }

    // Vbus test status
    if (controller.Vbus_cal_status != 0 && print_flag[2] == 0)
    {
      if (controller.Vbus_cal_status == 1)
      {
        Serialport.print("Vbus test: Failed");
        Serialport.println(" ");
      }
      else if (controller.Vbus_cal_status == 2)
      {
        Serialport.print("Vbus test: Passed");
        Serialport.println(" ");
      }
      print_flag[2] = 1;
    }

    // Resistance test status
    if (controller.Resistance_cal_status != 0 && print_flag[3] == 0)
    {
      if (controller.Resistance_cal_status == 1)
      {
        Serialport.print("Resistance test: Failed");
        Serialport.println(" ");
      }
      else if (controller.Resistance_cal_status == 2)
      {
        Serialport.print("Resistance = ");
        Serialport.print(controller.Resistance, 6);
        Serialport.print("Ohm");
        Serialport.println(" ");
      }
      print_flag[3] = 1;
    }

    // Inductance test status
    if (controller.Inductance_cal_status != 0 && print_flag[4] == 0)
    {
      if (controller.Inductance_cal_status == 1)
      {
        Serialport.print("Inductance test: Failed");
        Serialport.println(" ");
      }
      else if (controller.Inductance_cal_status == 2)
      {
        Serialport.print("Inductance = ");
        Serialport.print(controller.Inductance, 6);
        Serialport.print("H");
        Serialport.println(" ");

        Serialport.print("Kp_iq = ");
        Serialport.print(PID.Kp_iq, 6);
        Serialport.println(" ");

        Serialport.print("Ki_iq = ");
        Serialport.print(PID.Ki_iq, 6);
        Serialport.println(" ");
      }
      // Print Ki Kp za struje i zadani bandwidth
      print_flag[4] = 1;
    }

    // Open loop status
    if (controller.Open_loop_cal_status != 0 && print_flag[5] == 0)
    {
      if (controller.Open_loop_cal_status == 1)
      {
        Serialport.println("Open loop: Failed");
        Serialport.println("The rotor did not follow the open-loop field (pole pairs must come out as 1 to 49).");
        Serialport.println("Check the motor's three wires and that the shaft is free.");
      }
      else if (controller.Open_loop_cal_status == 2)
      {
        Serialport.print("Pole pair = ");
        Serialport.print(controller.pole_pairs);
        Serialport.println(" ");
        Serialport.print("Dir = ");
        Serialport.print(controller.DIR_);
        Serialport.println(" ");
      }
      // Settings used
      print_flag[5] = 1;
    }

    // Phase order test status
    if (controller.Phase_order_status != 0 && print_flag[6] == 0)
    {
      if (controller.Phase_order_status == 1)
      {
        Serialport.print("Phase order = ");
        Serialport.print(controller.Phase_order);
        Serialport.println(" ");
        // NOTE: this used to mean "phase rotation is wrong, swap two phases" -- that failure
        // mode no longer exists (reversed wiring is now auto-detected/compensated in software,
        // see commutation_dir). This status now means either the commutation-confirm spin
        // barely moved, or the angle-offset alignment couldn't confirm +Iq -> +encoder --
        // i.e. the motor did not respond as expected to a CURRENT command. Check wiring,
        // encoder, and current sensing (voltage-mode driving during R/L/pole-pair measurement
        // still worked, so this points at the closed-loop current path specifically).
        Serialport.println("Commutation/alignment check failed!");
        Serialport.println("Motor did not respond as expected to a current command. Check wiring/encoder/current sensing and try #Cal again.");
        // Diagnostic: only meaningful if this failure came from the "confirm commutation" spin
        // test (0 means it failed elsewhere, in the angle-offset align/sweep instead).
        if (controller.Spin_confirm_delta != 0)
        {
          Serialport.print("Spin-confirm delta: ");
          Serialport.print(controller.Spin_confirm_delta);
          Serialport.print(" ticks (need > ");
          Serialport.print(CPR / 2);
          Serialport.print("), Vbus: ");
          Serialport.print(controller.Spin_confirm_vbus_mV);
          Serialport.println(" mV");
        }
      }
      else if (controller.Phase_order_status == 2)
      {
        Serialport.print("Phase order = ");
        Serialport.print(controller.Phase_order);
        Serialport.println(" ");
        Serialport.println("Phase order is good!");
      }
      print_flag[6] = 1;
    }

    // Kt call
    if (controller.KV_status != 0 && print_flag[7] == 0)
    {
      if (controller.KV_status == 2)
      {
        Serialport.print("KV is: ");
        Serialport.print(controller.KV, 5);
        Serialport.println(" ");
        Serialport.print("Kt is: ");
        Serialport.print(controller.Kt, 5);
        Serialport.println(" ");
        Serialport.print("KV runs:");
        for (int k = 0; k < KV_RUNS; k++)
        {
          Serialport.print(" ");
          Serialport.print(controller.KV_runs[k], 2);
        }
        Serialport.print(" (same direction within ");
        Serialport.print(controller.KV_spread_pct, 1);
        Serialport.println(" %)");
        if (controller.KV_spread_pct > KV_SPREAD_WARN_PCT)
          Serialport.println("KV runs disagree: check that the shaft is free, then calibrate again");
        Serialport.print("Flux linkage is: ");
        Serialport.print(controller.flux_linkage, 6);
        Serialport.println(" ");
        Serialport.print("----------------");
      }

      print_flag[7] = 1;
    }

    // If calib failed
    // Set clalibration status to 0 (go outside of calibration) set calibrated flag to 0 and go idle.
    // Also report error.
    if (controller.Calibration == 2 && print_flag[8] == 0)
    {
      Serialport.print("Calibration failed!");
      Serialport.println(" ");
      if (controller.ESTOP_error)
        Serialport.println("Stopped by an E-stop: #Clear, then calibrate again.");
      Serialport.print("----------------");
      Serialport.println(" ");
      controller.Controller_mode = 0;
      // Settings used
      print_flag[8] = 1;
      Ticker_detach(TIM3);
      Ticker_init(TIM3, LOOP_FREQ, IT_callback);
      controller.Error = 0;
      controller.Calibrated = 0;

      controller.Calibration = 0;
      controller.Calib_error = 0;
      controller.Magnet_cal_status = 0;
      controller.Vbus_cal_status = 0;
      controller.Resistance_cal_status = 0;
      controller.Inductance_cal_status = 0;
      controller.Open_loop_cal_status = 0;
      controller.Kt_cal_status = 0;
      controller.Phase_order_status = 0;
      controller.KV_status = 0;
      for (int i = 0; i < sizeof(print_flag) / sizeof(print_flag[0]); i++)
      {
        print_flag[i] = 0; // Set each element to 0
      }
    }

    // If calib was success
    // Set clalibration status to 0 (go outside of calibration)
    // set calibrated flag to 1 and enter closed loop mode (Idle) if there are no errors.
    if (controller.Calibration == 3 && print_flag[9] == 0)
    {

      Serialport.print("Calibration success!");
      Serialport.println(" ");
      const uint8_t next = Encal_after_calibration_planned(); // both, or nothing after #Calonly
      if (next == AFTER_CAL_BOTH) // batch 20
        Serialport.println("Next, by itself: the encoder table, then anti-cogging (about a minute; keep the shaft "
                           "free). Everything is saved when they finish (#Encal state, #Cogcal state)");
      else if (next == AFTER_CAL_COGGING)
        Serialport.println("Next, by itself: anti-cogging (about 50 s; keep the shaft free). Everything is saved "
                           "when it finishes (#Cogcal state)");
      else
      {
        Serialport.print("Save config with #Save");
        Serialport.println(" ");
      }
      Serialport.print("Going to idle!");
      Serialport.println(" ");
      Serialport.print("----------------");
      Serialport.println(" ");
      controller.Controller_mode = 0;
      // Settings used
      print_flag[9] = 1;
      Ticker_detach(TIM3);
      Ticker_init(TIM3, LOOP_FREQ, IT_callback);
      controller.Error = 0;
      controller.Calibrated = 1;
      Cal_restore_current_limit(); // the user's limit again, before the runs that follow (batch 22d)
      Calibration_id_new(); // before those runs, whose save keeps it
      Encal_after_calibration(); // the encoder table and the anti-cogging follow (not after #Calonly)

      controller.Calibration = 0;
      controller.Calib_error = 0;
      controller.Magnet_cal_status = 0;
      controller.Vbus_cal_status = 0;
      controller.Resistance_cal_status = 0;
      controller.Inductance_cal_status = 0;
      controller.Open_loop_cal_status = 0;
      controller.Kt_cal_status = 0;
      controller.Phase_order_status = 0;
      controller.KV_status = 0;
      for (int i = 0; i < sizeof(print_flag) / sizeof(print_flag[0]); i++)
      {
        print_flag[i] = 0; // Set each element to 0
      }
    }
  }
}

/// @brief Exit sleep mode and Enable FET driver
void Enable_drive()
{
  // Sleep mode input.Logic high to enable device;
  // logic low to enter low-power sleep mode; internal pulldown
  // digitalWriteFast(SLEEP, HIGH);
  // Reset input. Active-low reset input initializes internal logic, clears faults,
  // and disables the outputs, internal pulldown
  // digitalWriteFast(RESET, HIGH);

  if (controller.reset_pin_state == 0 && controller.sleep_pin_state == 0)
  {
    digitalWriteFast(SLEEP, HIGH);
    digitalWriteFast(RESET, HIGH);
    controller.reset_pin_state = 1;
    controller.sleep_pin_state = 1;
  }
}

/// A position error this small or smaller counts as none, for the whole position loop: 0 (off) except while the
/// gripper holds a position it has reached. Gripper_mode() sets it for its own call and takes it back, so no
/// other mode ever sees it.
static int position_hold_deadzone = 0;

/// The current limit of the speed loop for this call, mA; -1: the drive's own (PID.Iq_current_limit, parameter 16).
/// Gripper mode and the gripper calibration set it to the grip current and take it back, so the drive's limit
/// stays what the user set. They used to write the grip current into the limit itself, and a #Save then
/// stored it as the drive's.
static float current_limit_override = -1.0f;

/// @todo feedforwards
/// @brief FOC cascaded position mode
/// @todo add cogging comp
void Position_mode()
{
  /* Positon PIDS*/
  float position_error = PID.Position_setpoint - controller.Position_Ticks;
  float speed_ff = PID.Feedforward_speed;
  Input_filter(PID.Position_setpoint, PID.Feedforward_speed, &position_error, &speed_ff); // parameter 74 (batch 18)
  if (position_hold_deadzone > 0 && position_error <= (float)position_hold_deadzone &&
      position_error >= -(float)position_hold_deadzone)
    position_error = 0.0f;
  PID.P_errSum = PID.Kp_p * position_error;
  PID.Velocity_setpoint = PID.P_errSum + speed_ff;

  /*Clamp velocity to velocity limit*/
  if (PID.Velocity_setpoint > PID.Velocity_limit)
    PID.Velocity_setpoint = PID.Velocity_limit;
  else if (PID.Velocity_setpoint < -PID.Velocity_limit)
    PID.Velocity_setpoint = -PID.Velocity_limit;

  /* Velocity PIDS*/
  float velocity_error = PID.Velocity_setpoint - controller.Velocity_Filter;
  // the integral's own speed error: with the deadband (parameter 75) the position error less the band (batch 18)
  const float integral_error = (controller.vel_int_deadband == 0) ? velocity_error
                               : Vel_int_setpoint(position_error, speed_ff) - controller.Velocity_Filter;
  PID.Iq_setpoint = Velocity_pi(velocity_error, integral_error); // clamped

  // PID.Iq_setpoint = -100;

  /* Current PIDS*/
  float Id_error = PID.Id_setpoint - FOC.Id;
  float Iq_error = PID.Iq_setpoint - FOC.Iq;

  PID.Id_errSum = PID.Id_errSum + Id_error * PID.Ki_id;
  PID.Iq_errSum = PID.Iq_errSum + Iq_error * PID.Ki_iq;

  int voltage_limit_var = controller.VBUS_mV;
  if (PID.Voltage_limit == 0)
  {
    voltage_limit_var = controller.VBUS_mV;
  }
  else if (PID.Voltage_limit > 0 && PID.Voltage_limit < controller.VBUS_mV)
  {
    voltage_limit_var = PID.Voltage_limit;
  }
  else
  {
    voltage_limit_var = controller.VBUS_mV;
  }

  /* Handle integral windup*/
  limit_norm(&PID.Id_errSum, &PID.Iq_errSum, voltage_limit_var);

  float Ud_setpoint = PID.Kp_id * Id_error + PID.Id_errSum;
  float Uq_setpoint = PID.Kp_iq * Iq_error + PID.Iq_errSum;

  /* Clamp outputs*/
  /***********************************/
  limit_norm(&Ud_setpoint, &Uq_setpoint, voltage_limit_var * OVERMODULATION);
  abc_fast(Ud_setpoint, Uq_setpoint, &FOC.U1, &FOC.U2, &FOC.U3);
  // sinusoidal_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);
  space_vector_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);

  FOC.PWM1 = FOC.U1_normalized * PWM_MAX;
  FOC.PWM2 = FOC.U2_normalized * PWM_MAX;
  FOC.PWM3 = FOC.U3_normalized * PWM_MAX;

  Phase_order();
}

/// @todo feedforwards
/// @brief  FOC cascade velocity mode
/// @todo add cogging comp
void Velocity_mode()
{

  /*Clamp velocity to velocity limit*/
  if (PID.Velocity_setpoint > PID.Velocity_limit)
    PID.Velocity_setpoint = PID.Velocity_limit;
  else if (PID.Velocity_setpoint < -PID.Velocity_limit)
    PID.Velocity_setpoint = -PID.Velocity_limit;

  /* Velocity PIDS*/
  float velocity_error = PID.Velocity_setpoint - controller.Velocity_Filter;
  PID.Iq_setpoint = Velocity_pi(velocity_error, velocity_error); // clamped

  // PID.Iq_setpoint = -100;

  /* Current PIDS*/
  float Id_error = PID.Id_setpoint - FOC.Id;
  float Iq_error = PID.Iq_setpoint - FOC.Iq;

  PID.Id_errSum = PID.Id_errSum + Id_error * PID.Ki_id;
  PID.Iq_errSum = PID.Iq_errSum + Iq_error * PID.Ki_iq;

  int voltage_limit_var = controller.VBUS_mV;
  if (PID.Voltage_limit == 0)
  {
    voltage_limit_var = controller.VBUS_mV;
  }
  else if (PID.Voltage_limit > 0 && PID.Voltage_limit < controller.VBUS_mV)
  {
    voltage_limit_var = PID.Voltage_limit;
  }
  else
  {
    voltage_limit_var = controller.VBUS_mV;
  }

  /* Handle integral windup*/
  limit_norm(&PID.Id_errSum, &PID.Iq_errSum, voltage_limit_var);

  float Ud_setpoint = PID.Kp_id * Id_error + PID.Id_errSum;
  float Uq_setpoint = PID.Kp_iq * Iq_error + PID.Iq_errSum;

  /* Clamp outputs*/
  /***********************************/
  limit_norm(&Ud_setpoint, &Uq_setpoint, voltage_limit_var * OVERMODULATION);
  abc_fast(Ud_setpoint, Uq_setpoint, &FOC.U1, &FOC.U2, &FOC.U3);
  // sinusoidal_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);
  space_vector_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);

  FOC.PWM1 = FOC.U1_normalized * PWM_MAX;
  FOC.PWM2 = FOC.U2_normalized * PWM_MAX;
  FOC.PWM3 = FOC.U3_normalized * PWM_MAX;

  Phase_order();
}

/// @todo feedforwards
/// @brief  FOC cascade torque/current mode
void Torque_mode()
{

  /* Clamp Iq to current limit for the control math only; PID.Iq_setpoint keeps the
     user's actual commanded value (e.g. reading back "#Iq" reflects what was asked
     for, not a value silently overwritten by this clamp). */
  float Iq_setpoint_clamped = PID.Iq_setpoint;
  if (Iq_setpoint_clamped > PID.Iq_current_limit)
    Iq_setpoint_clamped = PID.Iq_current_limit;
  else if (Iq_setpoint_clamped < -PID.Iq_current_limit)
    Iq_setpoint_clamped = -PID.Iq_current_limit;

  /* Current PIDS*/
  float Id_error = PID.Id_setpoint - FOC.Id;
  float Iq_error = Iq_setpoint_clamped - FOC.Iq;

  PID.Id_errSum = PID.Id_errSum + Id_error * PID.Ki_id;
  PID.Iq_errSum = PID.Iq_errSum + Iq_error * PID.Ki_iq;

  int voltage_limit_var = controller.VBUS_mV;
  if (PID.Voltage_limit == 0)
  {
    voltage_limit_var = controller.VBUS_mV;
  }
  else if (PID.Voltage_limit > 0 && PID.Voltage_limit < controller.VBUS_mV)
  {
    voltage_limit_var = PID.Voltage_limit;
  }
  else
  {
    voltage_limit_var = controller.VBUS_mV;
  }

  /* Handle integral windup*/
  limit_norm(&PID.Id_errSum, &PID.Iq_errSum, voltage_limit_var);

  float Ud_setpoint = (PID.Kp_id * Id_error + PID.Id_errSum);
  float Uq_setpoint = (PID.Kp_iq * Iq_error + PID.Iq_errSum);

  /* Clamp outputs*/
  /***********************************/
  limit_norm(&Ud_setpoint, &Uq_setpoint, (voltage_limit_var * OVERMODULATION));
  abc_fast(Ud_setpoint, Uq_setpoint, &FOC.U1, &FOC.U2, &FOC.U3);
  // sinusoidal_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);
  space_vector_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);

  FOC.PWM1 = FOC.U1_normalized * PWM_MAX;
  FOC.PWM2 = FOC.U2_normalized * PWM_MAX;
  FOC.PWM3 = FOC.U3_normalized * PWM_MAX;
  //

  Phase_order();
}

/// @todo feedforwards
/// @brief  FOC cascade torque/current mode
void Voltage_Torque_mode()
{

  float Ud_setpoint = PID.Ud_setpoint;
  float Uq_setpoint = PID.Uq_setpoint;

  /* Clamp outputs*/
  /***********************************/
  limit_norm(&Ud_setpoint, &Uq_setpoint, (controller.VBUS_mV * OVERMODULATION));
  abc_fast(Ud_setpoint, Uq_setpoint, &FOC.U1, &FOC.U2, &FOC.U3);
  // sinusoidal_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);
  space_vector_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);

  FOC.PWM1 = FOC.U1_normalized * PWM_MAX;
  FOC.PWM2 = FOC.U2_normalized * PWM_MAX;
  FOC.PWM3 = FOC.U3_normalized * PWM_MAX;
  //

  Phase_order();
}

/// @todo feedforwards
/// @brief Impedance PD controller
/// @todo add cogging comp
void PD_mode()
{
  // PID.Iq_setpoint = PID.KP *() + PID.KD*() + current_feedforward + cogging_comp
  float position_error = PID.Position_setpoint - controller.Position_Ticks;
  float speed_setpoint = PID.Velocity_setpoint;
  Input_filter(PID.Position_setpoint, PID.Velocity_setpoint, &position_error, &speed_setpoint); // parameter 74 (batch 18)
  PID.Iq_setpoint = PID.KP * position_error + PID.KD * (speed_setpoint - controller.Velocity_Filter) + PID.Feedforward_current;
  // PID.Iq_setpoint = PID.KP * (0 - controller.Position_Ticks) + PID.KD * (0 - controller.Velocity_Filter);

  /* Clamp Iq to current limit*/

  if (PID.Iq_setpoint > PID.Iq_current_limit)
    PID.Iq_setpoint = PID.Iq_current_limit;
  else if (PID.Iq_setpoint < -PID.Iq_current_limit)
    PID.Iq_setpoint = -PID.Iq_current_limit;

  // PID.Iq_setpoint = -100;

  /* Current PIDS*/
  float Id_error = PID.Id_setpoint - FOC.Id;
  float Iq_error = PID.Iq_setpoint - FOC.Iq;

  PID.Id_errSum = PID.Id_errSum + Id_error * PID.Ki_id;
  PID.Iq_errSum = PID.Iq_errSum + Iq_error * PID.Ki_iq;

  int voltage_limit_var = controller.VBUS_mV;
  if (PID.Voltage_limit == 0)
  {
    voltage_limit_var = controller.VBUS_mV;
  }
  else if (PID.Voltage_limit > 0 && PID.Voltage_limit < controller.VBUS_mV)
  {
    voltage_limit_var = PID.Voltage_limit;
  }
  else
  {
    voltage_limit_var = controller.VBUS_mV;
  }

  /* Handle integral windup*/
  limit_norm(&PID.Id_errSum, &PID.Iq_errSum, voltage_limit_var);

  float Ud_setpoint = (PID.Kp_id * Id_error + PID.Id_errSum);
  float Uq_setpoint = (PID.Kp_iq * Iq_error + PID.Iq_errSum);

  /* Clamp outputs*/
  /***********************************/
  limit_norm(&Ud_setpoint, &Uq_setpoint, (voltage_limit_var * OVERMODULATION));
  abc_fast(Ud_setpoint, Uq_setpoint, &FOC.U1, &FOC.U2, &FOC.U3);
  // sinusoidal_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);
  space_vector_commutation(controller.VBUS_mV, FOC.U1, FOC.U2, FOC.U3, &FOC.U1_normalized, &FOC.U2_normalized, &FOC.U3_normalized);

  FOC.PWM1 = FOC.U1_normalized * PWM_MAX;
  FOC.PWM2 = FOC.U2_normalized * PWM_MAX;
  FOC.PWM3 = FOC.U3_normalized * PWM_MAX;

  Phase_order();
}

/// @brief How LED acts depending on control mode and errors
/// @param ms clock with ms resolution
void LED_status(uint32_t ms)
{
  static uint32_t last_time = 0;
  static uint32_t last_time_LED = 0;
  static uint32_t BLINK_cnt = 0;
  static uint32_t BLINK_delay_cnt = 0;
  static int led_state = 0;

  if (controller.Calibration == 1)
  {
    if ((ms - last_time_LED) > 50) // run every x ms
    {
      led_state = !led_state;
      digitalWriteFast(LED, led_state);
      last_time_LED = ms;
    }
  }

  else if (controller.Error == 1)
  {
    digitalWriteFast(LED, HIGH);
  }

  else if (controller.Error == 0 && controller.Calibrated == 1)
  {
    if (BLINK_cnt <= 6)
    {
      if ((ms - last_time_LED) > 115) // run every x ms
      {
        led_state = !led_state;
        digitalWriteFast(LED, led_state);
        last_time_LED = ms;
        BLINK_cnt = BLINK_cnt + 1;
      }
    }
    else
    {
      digitalWriteFast(LED, LOW);
      if ((ms - last_time_LED) > 400) // run every x ms
      {

        digitalWriteFast(LED, LOW);
        last_time_LED = ms;
        BLINK_cnt = 0;
      }
    }
  }
}

/// @brief Encoder parity check. Not used at this moment.
/// @param data
/// @return
bool parityCheck(uint16_t data)
{
  data ^= data >> 8;
  data ^= data >> 4;
  data ^= data >> 2;
  data ^= data >> 1;

  return (~data) & 1;
}

/// @brief Get current in mA from RAW encoder ticks. 0A is equal to ADC_MIDPOINT
/// @param adc_value RAW ADC value
/// @return Current in mA
int Get_current_mA(int adc_value)
{

  // Formula is Vout = Vref + (Vsense * GAIN)
  // int current = qfp_fmul((adc_value - 2048), CURRENT_SENSE_CONSTANT_mV);

  int current = qfp_fmul((adc_value - 2048), CURRENT_SENSE_CONSTANT_mV);
  return current;
}

int Get_ADC_Value(int current_mA)
{
  // Rearranged formula: adc_value = (current / CURRENT_SENSE_CONSTANT_mV) + 2048
  int adc_value = (current_mA / CURRENT_SENSE_CONSTANT_mV) + 2048;

  // Clamp the result within the range of a 12-bit ADC
  adc_value = constrain(adc_value, 0, 4095);

  return adc_value;
}

/// @brief Get current in A from RAW encoder ticks. 0A is equal to ADC_MIDPOINT
/// @param adc_value RAW ADC value
/// @return Current in A
float Get_current(int adc_value)
{

  // Formula is Vout = Vref + (Vsense * GAIN)
  float current = qfp_fmul((adc_value - 2048), CURRENT_SENSE_CONSTANT);

  return current;
}

/// @brief Max Voltage is 40V, 10K and 870 ohm resistor in series. For 3v3 on ADC pin we need 41.231V on input.
/// @brief 41231 mV = 4095 ADC ticks => (41231/4095) = 10.06 = 10. Voltage_mv = ADC_tick * 10
/// @param adc_value RAW ADC value
/// @return Voltage in mv
int Get_voltage_mV(int adc_value)
{

  int voltage = qfp_fmul(adc_value, 10);
  return voltage;
}

void Phase_order()
{
  // Reversed-wiring compensation (software reflection): swap phases B<->C (PWM2<->PWM3) when
  // commutation_dir < 0, matching the Sense2<->Sense3 swap in Collect_data(). Together they form
  // a reflection that cancels a physical phase reversal, so a reversed-wired motor becomes a
  // normally-wired (proper-rotation) one and runs the exact same control path as forward wiring.
  // The base routing below (DIR_/Phase_order) is the fixed hardware frame; commutation_dir handles
  // the wiring handedness and theta_offset (from Calibrate_Angle_Offset_Align) handles the rotation.
  // (bench-confirm: verify B<->C here + in Collect_data gives reversed wiring the same symmetric
  //  calibration as forward, with +Iq -> +encoder)
  int pwm_a = FOC.PWM1;
  int pwm_b = FOC.PWM2;
  int pwm_c = FOC.PWM3;
  if (controller.commutation_dir < 0)
  {
    int tmp = pwm_b;
    pwm_b = pwm_c;
    pwm_c = tmp;
  }

  if (controller.Phase_order == 2)
  {
    if (controller.DIR_ == 0)
    {

      pwm_set(PWM_CH1, pwm_a, 13);
      pwm_set(PWM_CH3, pwm_b, 13);
      pwm_set(PWM_CH2, pwm_c, 13);
    }
    else
    {
      pwm_set(PWM_CH3, pwm_a, 13);
      pwm_set(PWM_CH1, pwm_b, 13);
      pwm_set(PWM_CH2, pwm_c, 13);
    }
  }
}

/// @brief  Collect all sensor data2
void Collect_data2()
{

  uint16_t result;
  uint16_t result1;
  uint16_t result2;

  /*Get position, current1, current2, vbus*/
  /*position sensor packs its data in 2 8 bit data packets */
  /*First we send 0x8300 as a response we get one packet*/
  /*Then we send 0c8400 as a response we get second packet*/
  /*Between requesting packets we need small delay and we use ADC readings as a delay*/
  /*Reading sense1,2 and vbus takes around 10us */
  /*After we get both packets they are unpacked and we get position value as 14 bit, no mag bit and parity check bit */
  SPI.beginTransaction(MT6816_settings);
  digitalWriteFast(CSN, LOW);
  result1 = SPI.transfer16(0x8300);
  digitalWriteFast(CSN, HIGH);
  Sense_sample_wait();
  if (controller.DIR_ == 1)
  {
    controller.Sense1_Raw = ADC_CHANNEL_4_READ_SENSE1() - controller.sense1_offset;
    controller.Sense2_Raw = ADC_CHANNEL_3_READ_SENSE2() - controller.sense2_offset;
  }
  else
  {
    controller.Sense1_Raw = ADC_CHANNEL_3_READ_SENSE2() - controller.sense2_offset;
    controller.Sense2_Raw = ADC_CHANNEL_4_READ_SENSE1() - controller.sense1_offset;
  }

  controller.VBUS_RAW = ADC_CHANNEL_6_READ_VBUS();
  result1 = result1 << 8;
  digitalWriteFast(CSN, LOW);
  result2 = SPI.transfer16(0x8400);
  digitalWriteFast(CSN, HIGH);
  result = result1 | result2;
  controller.Position_Raw = Encoder_correct(result >> 2); // the encoder table, when in use
  SPI.endTransaction();
  /***********************************/

  /* Check for magnet on MT6816*/
  if ((result & MT6816_NO_MAGNET_BIT) == MT6816_NO_MAGNET_BIT)
  { // no magnet or not detecting enough flux density
    controller.Magnet_warrning = true;
    controller.encoder_error = 1;
  }
  else
  { // Detected magnet
    controller.Magnet_warrning = false;
  }
  /***********************************/

  /* Parity check, works wierd and inconsistent, skip for now
 if(parityCheck(result)){
    controller.MT6816_parity_check_pass = 1;
 }else{
    controller.MT6816_parity_check_pass = 0;
 }
*/

  /* Transform RAW current and voltage to mV, mA.*/
  /* Ia + Ib + Ic = 0. Calculate Sense3 .*/
  controller.Sense1_mA = Get_current_mA(controller.Sense1_Raw);
  controller.Sense2_mA = Get_current_mA(controller.Sense2_Raw);

  controller.Sense3_mA = -controller.Sense1_mA - controller.Sense2_mA;

  // Reversed-wiring compensation (software reflection): swap phase-B/phase-C currents
  // (Sense2 <-> Sense3) when commutation_dir < 0, matching the PWM2<->PWM3 swap in Phase_order().
  // Must be identical to the swap in Collect_data() so calibration and the run loop agree.
  if (controller.commutation_dir < 0)
  {
    int tmp = controller.Sense2_mA;
    controller.Sense2_mA = controller.Sense3_mA;
    controller.Sense3_mA = tmp;
  }

  controller.VBUS_mV = Get_voltage_mV(controller.VBUS_RAW);
  /***********************************/

  /* Handle encoder full rotation overflow*/
  if (controller.Position_Raw - controller.Old_Position_Raw > CPR2)
  {
    controller.ROTATIONS = controller.ROTATIONS - 1;
  }
  else if (controller.Position_Raw - controller.Old_Position_Raw < -CPR2)
  {
    controller.ROTATIONS = controller.ROTATIONS + 1;
  }
  /***********************************/

  /* Motor position with multiple rotations in encoder ticks*/
  controller.Position_Ticks = controller.Position_Raw + (controller.ROTATIONS * CPR);
  /***********************************/

  /* Calculate velocity in ticks/s */
  controller.Velocity = (controller.Position_Ticks - controller.Old_Position_Ticks) * LOOP_FREQ;
  /* The speed the loops use: the moving average or the PLL (parameter 50) */
  controller.Velocity_Filter = Speed_estimate(controller.Velocity, controller.Position_Ticks - controller.Old_Position_Ticks);
  /***********************************/

  /*  Save position values for next cycle*/
  controller.Old_Position_Ticks = controller.Position_Ticks;
  controller.Old_Position_Raw = controller.Position_Raw;
  /***********************************/

  // Position_RAD = qfp_fmul(Position_Raw, RAD_CONST);

  /* Caclculate electrical angle */
  // controller.Electric_Angle = RAD_CONST * fmod((controller.Position_Raw * NPP),CPR);
  controller.Electric_Angle = RAD_CONST * ((controller.Position_Raw * controller.pole_pairs) % CPR);

  // Normalize angle between 0 and 2Pi
  if (controller.Electric_Angle < 0)
    controller.Electric_Angle = (controller.Electric_Angle + PI2);
  else if (controller.Electric_Angle > PI2)
    controller.Electric_Angle = (controller.Electric_Angle - PI2);
  /***********************************/

  /****  End measurment here *********/
}

#define POSITION_TOLERANCE 5              // ticks: within this of the target a move has arrived
#define VELOCITY_CONTACT_THRESHOLD 500.0f // ticks/s: slower than this counts as standing still
#define CURRENT_CONTACT_RATIO 0.8f        // of the grip current: more than this, standing still, is a stall
// While the gripper holds a position it has reached, a position error this small counts as none (ticks; one
// position count is about 40 to 170 ticks, so the position it reports does not change). Friction holds the jaws
// there. Without it the speed loop's integral wound up against the friction for a second or more to move the
// last ticks, and kept that current afterwards.
#define GRIPPER_HOLD_DEADZONE 16
// Once holding, the jaws count as inside until they are this far off (ticks). With one limit, jaws resting a
// tick outside it went in and out with the encoder's noise, and the integral was dropped each time: a current
// that sawed between 0 and 100 mA and never moved them (bench 2026-11-13).
#define GRIPPER_HOLD_RELEASE 32
// The last stretch of a move slows down: the speed is at most the slowest speed plus this many ticks/s per
// tick left to go (1/s). At full speed the jaws arrived at 60000 ticks/s, ran 200 ticks past the target and
// crept back for a second; now they arrive at the slowest speed whatever the speed of the move.
#define GRIPPER_APPROACH_GAIN 30
// A move picks up speed from the slowest speed: the whole range of speeds in this many loops (0.1 s). Before,
// the speed loop was asked for the full speed at once and answered with the full grip current for the first
// 30 ms of every move (a click).
#define GRIPPER_START_RAMP_LOOPS (LOOP_FREQ / 10)
// A stall is reported as an object only once it has lasted this long (control loops: 50 ms). Before, the first
// loops of a move from standstill, slow and already at the grip current, were reported as an object.
#define GRIPPER_STALL_CONFIRM (LOOP_FREQ / 20)

// The gripper calibration (Calibrate_gripper)
#define GRIPPER_CAL_CURRENT 500                    // mA of the push into each end
#define GRIPPER_CAL_SPEED 40                       // of 255. Lower this if it does not home
#define GRIPPER_CAL_STALL_CYCLES 3200              // loops standing still at the push current: the end is found
#define GRIPPER_CAL_RELEASE_CYCLES (LOOP_FREQ / 4) // loops without current after the push, before the end is read
#define GRIPPER_CAL_TIMEOUT_CYCLES 150000          // the whole run at most
#define GRIPPER_END_INSET 32                       // ticks that positions 0 and 255 keep clear of the stops
#define GRIPPER_MIN_STROKE 1000                    // ticks: less travel than this between the ends is no calibration

/// @brief Whether the jaws stand still while the drive pushes with nearly the grip current, and have done so
/// for GRIPPER_STALL_CONFIRM loops: an object (or a stop). *cycles counts the loops the stall has lasted.
static bool Gripper_stalled(int *cycles)
{
  const bool stall = fabs(controller.Velocity_Average) < VELOCITY_CONTACT_THRESHOLD &&
                     fabs(FOC.Iq) > CURRENT_CONTACT_RATIO * current_limit_override; // the grip current
  if (!stall)
    *cycles = 0;
  else if (*cycles < GRIPPER_STALL_CONFIRM)
    (*cycles)++;
  return *cycles >= GRIPPER_STALL_CONFIRM;
}

/// @brief Gripper mode
void Gripper_mode()
{
  static bool holding = 0;     // at the position and inside the hold dead zone on the last loop
  static int stall_cycles = 0; // loops the stall has lasted (Gripper_stalled)
  static int ramp_speed = 0;     // the speed asked of the speed loop while a move picks up speed, ticks/s
  static int ramp_direction = 0; // the way that move goes: 1 opening, -1 closing, 0 no move under way

  // --- Reset position flag if a new command was received ---
  const bool new_command = Gripper.Same_command == 0;
  if (new_command)
  {
    Gripper.At_position = 0; // Only reset here when new command arrives
    holding = 0;
  }

  // 0 to the drive's maximum. The frame carries a signed number: a negative one turned the current clamps round, so
  // they pinned the current at full whatever the loop asked for, and a large one was no limit at all.
  {
    const int grip_limit = abs(Gripper.current_setpoint);
    // the gripper's own limit for this loop; the drive's limit (parameter 16) stays as it is
    current_limit_override = grip_limit > MAX_DRIVE_CURRENT ? MAX_DRIVE_CURRENT : grip_limit;
  }
  PID.Position_setpoint = map(Gripper.position_setpoint, 0, 255, Gripper.max_open_position, Gripper.max_closed_position);
  int vel_setpoint = map(Gripper.speed_setpoint, 0, 255, Gripper.min_speed, Gripper.max_speed);
  {
    // slower over the last stretch (GRIPPER_APPROACH_GAIN), never slower than the slowest speed
    const long to_go = labs((long)PID.Position_setpoint - (long)controller.Position_Ticks);
    const long approach = (long)Gripper.min_speed + GRIPPER_APPROACH_GAIN * to_go;
    if (approach < (long)vel_setpoint)
      vel_setpoint = (int)approach;
  }

  // which way the target lies: 1 opening (the ticks go up), -1 closing, 0 there
  const int direction = (PID.Position_setpoint > controller.Position_Ticks) ? 1
                        : (PID.Position_setpoint < controller.Position_Ticks) ? -1 : 0;

  if (Gripper.action_status == 1 && Gripper.calibrated && Gripper.activated && controller.I_AM_GRIPPER)
  {
    Driver_wake(); // the active gripper drives the bridge; the idle branch below puts the driver to sleep
    int pos_error = abs(PID.Position_setpoint - controller.Position_Ticks);

    // ✅ If within tolerance, mark as at position (and never unset again). A new command for where the jaws
    // already rest (inside what the hold allows) is there at once: no move for a few ticks.
    if (pos_error < POSITION_TOLERANCE || (new_command && pos_error <= GRIPPER_HOLD_RELEASE))
    {
      Gripper.At_position = 1;
    }

    if (Gripper.At_position)
    {
      // Arrived: hold. The move ran in the speed loop, whose integral holds the current the jaws needed on the
      // way (the rail's friction). Carried into the hold it had to unwind through the friction, at a rate set by
      // the few ticks left: a second or more of current after every move, and a current that stayed (bench
      // 2026-11-13, MSG). So the integral is dropped when the jaws arrive, and again whenever they come back
      // inside the dead zone; while they hold, the position error counts as none and friction keeps them.
      const bool inside = pos_error <= (holding ? GRIPPER_HOLD_RELEASE : GRIPPER_HOLD_DEADZONE);
      if (inside && !holding)
        PID.V_errSum = 0;
      holding = inside;
      position_hold_deadzone = holding ? GRIPPER_HOLD_RELEASE : 0; // outside it the whole error pulls them back
      Position_mode();
      position_hold_deadzone = 0;
      Gripper.object_detection_status = 3; // at position
      stall_cycles = 0;
      ramp_direction = 0;
    }
    else
    {
      // Soft start: a move, or a turn the other way, begins at the slowest speed and picks up to its own
      // (GRIPPER_START_RAMP_LOOPS). A changed command the same way keeps the speed it has, so a host that
      // streams positions is not slowed down.
      if (direction != ramp_direction)
      {
        ramp_direction = direction;
        ramp_speed = Gripper.min_speed;
      }
      ramp_speed += (Gripper.max_speed - Gripper.min_speed) / GRIPPER_START_RAMP_LOOPS + 1;
      if (ramp_speed > vel_setpoint)
        ramp_speed = vel_setpoint;
      if (direction != 0)
        PID.Velocity_setpoint = direction * ramp_speed;
      Velocity_mode();
      holding = 0;
      if (Gripper_stalled(&stall_cycles))
        Gripper.object_detection_status = (FOC.Iq > 0) ? 2 : 1; // an object: 2 while opening, 1 while closing
      else
        Gripper.object_detection_status = 0; // moving, or not standing still for long enough yet
    }
  }
  else
  {
    // --- Idle or disabled ---
    PID.Velocity_setpoint = 0;
    Velocity_mode();
    holding = 0;
    ramp_direction = 0;

    // Sleep mode input.Logic high to enable device;
    // logic low to enter low-power sleep mode; internal pulldown
    digitalWriteFast(SLEEP, LOW);
    // Reset input. Active-low reset input initializes internal logic, clears faults,
    // and disables the outputs, internal pulldown
    digitalWriteFast(RESET, LOW);
    controller.reset_pin_state = 0;
    controller.sleep_pin_state = 0;

    if (Gripper_stalled(&stall_cycles))
      Gripper.object_detection_status = (FOC.Iq > 0) ? 2 : 1;
    else
      Gripper.object_detection_status = 0;
  }

  Gripper.Same_command = 1;
  current_limit_override = -1.0f;
}

/// @brief A gripper calibration that did not finish (idle, E-stop or an error came first). The ends of the last
/// calibration stay. The drive's current limit needs nothing: the calibration pushes with its own.
void Gripper_calibration_abort()
{
  current_limit_override = -1.0f;
  Gripper.In_calibration = 0;
}

/// @brief Gripper calibration (CAN 62, #Gripcal): find both ends of the stroke.
/// The jaws are pushed open at GRIPPER_CAL_CURRENT until they stand still, the push is released so they come to
/// rest against the stop, and that position is read; then the same closing. Positions 0 (open) and 255 (closed)
/// are set GRIPPER_END_INSET ticks inside the two resting positions, so a move to either end arrives in free
/// travel. Until 2026-11 the ends were read during the push: 0 and 255 were then positions inside the stops,
/// which a move with less current than the push never reached; it pressed on the stop for good and reported an
/// object (bench 2026-11-13, MSG: the open end 16 ticks short at 300 mA, with 5 ticks allowed).
/// Sets Gripper.calibrated: 1 when both ends were found and lie at least GRIPPER_MIN_STROKE apart, 0 otherwise
/// (also after GRIPPER_CAL_TIMEOUT_CYCLES). It pushes with its own current limit; the drive's (parameter 16) is
/// not touched.
void Calibrate_gripper()
{
  enum
  {
    OPEN_PUSH,
    OPEN_RELEASE,
    CLOSE_PUSH,
    CLOSE_RELEASE
  };
  static int phase = OPEN_PUSH;
  static int run_cycles = 0;         // loops of the whole run, for the timeout
  static int phase_cycles = 0;       // a push: loops standing still without a break; a release: loops since the push
  static int open_stop = 0;          // where the jaws rest against the open stop

  // A new run starts from its first step. A run that was cut short (idle, E-stop, an error) kept its step and
  // its counters, and the next one went on from there.
  if (Gripper.cal_restart)
  {
    Gripper.cal_restart = 0;
    phase = OPEN_PUSH;
    run_cycles = 0;
    phase_cycles = 0;
  }
  Gripper.In_calibration = 1;
  run_cycles++;

  bool finished = false;
  const int push_speed = map(GRIPPER_CAL_SPEED, 0, 255, Gripper.min_speed, Gripper.max_speed);

  switch (phase)
  {
  case OPEN_PUSH:
  case CLOSE_PUSH:
    current_limit_override = GRIPPER_CAL_CURRENT;
    PID.Velocity_setpoint = (phase == OPEN_PUSH) ? push_speed : -push_speed;
    Velocity_mode();
    /// If it is not moving and current is around the setpoint
    if (isAroundValue(abs(controller.Velocity_Average), 0, 700) &&
        isAroundValue(abs(FOC.Iq), GRIPPER_CAL_CURRENT, 90))
      phase_cycles++;
    else
      phase_cycles = 0;
    if (phase_cycles >= GRIPPER_CAL_STALL_CYCLES)
    {
      phase = (phase == OPEN_PUSH) ? OPEN_RELEASE : CLOSE_RELEASE;
      phase_cycles = 0;
    }
    break;

  case OPEN_RELEASE:
  case CLOSE_RELEASE:
    // No current: the jaws come to rest where the stop is, not where the push pressed them into it.
    PID.Iq_setpoint = 0;
    Torque_mode();
    PID.V_errSum = 0; // the next push starts without the last one's integral
    phase_cycles++;
    if (phase_cycles >= GRIPPER_CAL_RELEASE_CYCLES)
    {
      phase_cycles = 0;
      if (phase == OPEN_RELEASE)
      {
        open_stop = controller.Position_Ticks;
        phase = CLOSE_PUSH;
      }
      else
      {
        const int closed_stop = controller.Position_Ticks;
        // Opening is the positive direction, so the open stop is the larger number.
        if (open_stop - closed_stop >= GRIPPER_MIN_STROKE)
        {
          Gripper.max_open_position = open_stop - GRIPPER_END_INSET;
          Gripper.max_closed_position = closed_stop + GRIPPER_END_INSET;
          Gripper.calibrated = 1;
        }
        else
        {
          Gripper.calibrated = 0; // the jaws did not travel: jammed, or nothing moves them
        }
        finished = true;
      }
    }
    break;
  }

  current_limit_override = -1.0f; // the push current was for this loop only

  /// Calibration timed out
  /// during whole calibration we keep track of the loops; one loop is equal to interrupt time.
  if (!finished && run_cycles >= GRIPPER_CAL_TIMEOUT_CYCLES)
  {
    Gripper.calibrated = 0;
    finished = true;
  }

  /// At the end reset everything
  if (finished)
  {
    phase = OPEN_PUSH;
    run_cycles = 0;
    phase_cycles = 0;
    PID.V_errSum = 0;
    // digitalWriteFast(SLEEP, LOW);
    // digitalWriteFast(RESET, LOW);
    controller.reset_pin_state = 0;
    controller.sleep_pin_state = 0;
    controller.Controller_mode = 0;
    Gripper.In_calibration = 0;
  }
}

/// @brief Current-loop PI gains from the measured phase resistance and inductance at
/// controller.current_control_bandwidth (Hz), the standard way: the PI's zero
/// cancels the coil's R/L pole, so the closed loop is first order at that bandwidth (Kp = w L, Ki = w R; here in the
/// discrete form, Ki per loop = 1 - exp(-R T / L)). Until 2026-11 the STEPFOC divided both by 7 ("found
/// empirically"), a 71 Hz loop. Returns false and leaves the gains alone if R or L is not measured.
bool Current_gains_from_RL()
{
  const float R = controller.Resistance;
  const float L = controller.Inductance;
  if (!(R > 0.0f) || !(L > 0.0f))
    return false;
  const float ki = 1.0f - qfp_fexp(-R * (float)LOOP_TIME / L);
  const float crossover = (float)controller.current_control_bandwidth / (float)(LOOP_FREQ / PI2);
  const float kp = R * crossover / ki;
  if (!(kp > 0.0f) || !(ki > 0.0f))
    return false;
  PID.Ki = ki;
  controller.Crossover_frequency = crossover;
  PID.Kp_iq = kp;
  PID.Ki_iq = ki * kp;
  PID.Kp_id = kp;
  PID.Ki_id = ki * kp;
  return true;
}

/// @brief The bandwidth, Hz, that the current gains in force give with the measured R and L (the inverse of
/// Current_gains_from_RL), or 0 if R, L or Kp_iq is not set.
float Current_bandwidth_in_force()
{
  const float R = controller.Resistance;
  const float L = controller.Inductance;
  if (!(R > 0.0f) || !(L > 0.0f) || !(PID.Kp_iq > 0.0f))
    return 0.0f;
  const float ki = 1.0f - qfp_fexp(-R * (float)LOOP_TIME / L);
  return PID.Kp_iq * ki / R * (float)(LOOP_FREQ / PI2);
}

/// Speed estimate (firmware 2026-11 batch 8). Parameter 50 picks the one the loops use:
///   0  the vendor's 20-sample moving average of the count difference (3.2 ms at 6250 Hz: about 1.5 ms late, in
///      steps of LOOP_FREQ / 20 ticks/s)
///   1  a PLL: kp = 2 w, ki = w^2, critically damped; the speed is the integrator,
///      a smooth second-order low-pass (about 2 / w late); snaps to zero at standstill
///   2  the direct PLL: the same loop, but the speed is integrator + kp x error: almost no lag, more
///      of the count's quantisation noise
/// w = 2 pi x parameter 51 (Hz). The moving average runs every loop; a PLL starts from it when picked (bumpless).
/// The PLLs are written on the error between the encoder and the estimate, so the floats stay small anywhere.
// In integers (batch 13): positions in 1/65536 count, speeds in 1/16 tick/s. The float version cost 8.6 us per loop
// (over 1 us per soft-float operation on this chip); this one gives the same estimate (checked offline).
static int32_t pll_err = 0;   // encoder count minus the estimate's position, 1/65536 count
static int32_t pll_int = 0;   // the integrator, 1/16 tick/s
static int32_t pll_out = 0;   // the speed the PLL gives, 1/16 tick/s
static int32_t pll_kp_x = 0;  // kp x 256: (kp_x x delta) >> 20 is kp x delta in 1/16 tick/s
static int32_t pll_ki_x = 0;  // ki T x 256, the same scaling
static int32_t pll_k1_x = 0;  // (1 - kp T) x 65536
static int32_t pll_snap = 0;  // the snap-to-zero band (0.5 ki T), 1/16 tick/s
static uint8_t pll_kind = SPEED_ESTIMATOR_AVERAGE;
#define PLL_KT_X ((int32_t)((4096LL * 65536LL + LOOP_FREQ / 2) / LOOP_FREQ)) // a speed x T in 1/65536 count, x 65536

/// @brief The PLLs' gains from controller.speed_pll_bandwidth (Hz, the natural frequency).
void Speed_pll_gains()
{
  const float w = PI2 * (float)controller.speed_pll_bandwidth;
  const float kp = 2.0f * w;
  const float kiT = w * w * (float)LOOP_TIME;
  pll_kp_x = (int32_t)(kp * 256.0f + 0.5f);
  pll_k1_x = (int32_t)((1.0f - kp * (float)LOOP_TIME) * 65536.0f + 0.5f);
  pll_snap = (int32_t)(kiT * 8.0f + 0.5f);
  pll_ki_x = (int32_t)(kiT * 256.0f + 0.5f); // last: non-zero marks the gains as set
}

/// @brief One loop of the speed estimate parameter 50 picks, ticks/s.
/// @param velocity_raw the count difference times LOOP_FREQ (controller.Velocity)
/// @param d_ticks the count difference since the last loop
int Speed_estimate(int velocity_raw, int d_ticks)
{
  const int average = movingAverage(velocity_raw);
  controller.Velocity_Average = average;
  const uint8_t kind = controller.speed_estimator;
  if (kind != SPEED_ESTIMATOR_PLL && kind != SPEED_ESTIMATOR_PLL_DIRECT)
  {
    pll_kind = SPEED_ESTIMATOR_AVERAGE;
    return average;
  }
  if (pll_ki_x == 0)
    Speed_pll_gains();
  if (kind != pll_kind) // just picked: start from the average
  {
    pll_err = 0;
    pll_int = average * 16;
    pll_out = pll_int;
    pll_kind = kind;
  }
  if (d_ticks > SPEED_PLL_JUMP || d_ticks < -SPEED_PLL_JUMP)
  {
    pll_err = 0; // a position reset, not motion
  }
  else
  {
    const int32_t delta = pll_err + (int32_t)d_ticks * 65536 - (int32_t)(((int64_t)pll_out * PLL_KT_X) >> 16);
    pll_int += (int32_t)(((int64_t)pll_ki_x * delta + (1 << 19)) >> 20);
    if (kind == SPEED_ESTIMATOR_PLL)
    {
      pll_err = (int32_t)(((int64_t)delta * pll_k1_x) >> 16);
      if (pll_int < pll_snap && pll_int > -pll_snap)
        pll_int = 0; // snap to zero, so the count's dither at standstill does not jitter the speed
      pll_out = pll_int;
    }
    else
    {
      pll_err = delta;
      pll_out = pll_int + (int32_t)(((int64_t)pll_kp_x * delta + (1 << 19)) >> 20);
    }
  }
  return (pll_out + 8) >> 4;
}

/// @brief Id, Iq and the inverse transform's sin/cos at this electrical angle instead of the encoder's (encal.cpp:
/// the open loop that measures the encoder table).
void Foc_frame(float theta)
{
  dq0_abc_variables(theta);
  dq0_fast_int(controller.Sense1_mA, controller.Sense2_mA, controller.Sense3_mA, &FOC.Id, &FOC.Iq);
}

/// @brief The current-sense zero errors behind a 1st harmonic in the anti-cogging set (cogcal.cpp's trim). Not yet
/// on the Spectral: its three phases and the channel swaps of the phase order need their own map, to be measured on
/// its bench as sense_check.py did on the STEPFOC. Returns false: the trim is skipped.
bool Sense_trim_from_h1(float a, float b, int32_t *d1, int32_t *d2)
{
  (void)a;
  (void)b;
  *d1 = 0;
  *d2 = 0;
  return false;
}

/// @brief Commutation delay compensation (batch 12, parameter 64): the voltage this loop sets reaches the motor about
/// comm_delay_us later (the rest of the interrupt, the wait for the PWM update, half a control period), when the rotor
/// has turned on by speed x delay. The output transform rotates (d, q) forward by that angle. Small
/// angles: cos and sin by their series (clamped at 0.8 rad). Only in the closed-loop modes.
void Comm_delay_update()
{
  // the rotation per tick/s of speed, recomputed only when parameter 64 or the pole pairs change (batch 17)
  static uint16_t k_us = 0xFFFFu;
  static int k_pp = -1;
  static float k = 0.0f;
  if (controller.comm_delay_us != k_us || controller.pole_pairs != k_pp)
  {
    k_us = controller.comm_delay_us;
    k_pp = controller.pole_pairs;
    k = PI2 * (float)k_pp / (float)CPR * ((float)k_us * 1e-6f);
  }
  const int mode = controller.Controller_mode;
  const int v = controller.Velocity_Filter;
  if (k_us == 0 || mode < 1 || mode > 4 || (v < COMM_DELAY_MIN_SPEED && v > -COMM_DELAY_MIN_SPEED))
  {
    FOC.delay_c = 1.0f;
    FOC.delay_s = 0.0f; // no rotation: the output transform skips it
    return;
  }
  float dlt = (float)v * k;
  if (dlt > 0.8f)
    dlt = 0.8f;
  else if (dlt < -0.8f)
    dlt = -0.8f;
  const float d2 = dlt * dlt;
  FOC.delay_c = 1.0f - 0.5f * d2;
  FOC.delay_s = dlt * (1.0f - d2 * (1.0f / 6.0f));
}

/// ---- Velocity-loop details (batch 18), each off by default: parameters 74, 75 (RAM) ----

/// @brief The velocity loop's PI: the Iq setpoint, clamped to the current limit (position and velocity modes); the
/// integral sums and is clamped at the current limit. err_p is the speed error of the proportional part, err_i the one
/// the integral sums (they differ only with the deadband of parameter 75). Batch 18's anti-windup decay (parameter 73)
/// was removed in batch 19: on the bench it made a saturated speed step worse and changed nothing in moves or stalls.
float Velocity_pi(float err_p, float err_i)
{
  const float limit = current_limit_override >= 0.0f ? current_limit_override : PID.Iq_current_limit;
  const float ff = PID.Feedforward_current + Cogging_ff_mA();
  PID.V_errSum = PID.V_errSum + err_i * PID.Ki_v;
  if (PID.V_errSum > limit)
    PID.V_errSum = limit;
  else if (PID.V_errSum < -limit)
    PID.V_errSum = -limit;
  float iq = PID.Kp_v * err_p + PID.V_errSum + ff;
  if (iq > limit)
    iq = limit;
  else if (iq < -limit)
    iq = -limit;
  return iq;
}

/// @brief The speed setpoint the velocity integral sees in position mode with the deadband of parameter 75:
/// the position error less the band, zero inside it; clamped like the setpoint.
float Vel_int_setpoint(float position_error, float speed_ff)
{
  const float band = (float)controller.vel_int_deadband;
  float e = 0.0f;
  if (position_error > band)
    e = position_error - band;
  else if (position_error < -band)
    e = position_error + band;
  float v = PID.Kp_p * e + speed_ff;
  if (v > PID.Velocity_limit)
    v = PID.Velocity_limit;
  else if (v < -PID.Velocity_limit)
    v = -PID.Velocity_limit;
  return v;
}

static int64_t if_pos_q = 0;     // the filtered position, 1/65536 tick (float would lose the small steps far out)
static float if_vel = 0.0f;      // the filtered speed, ticks/s
static float if_kp = 0.0f;       // w^2
static float if_ki = 0.0f;       // 2 w: critically damped
static uint16_t if_hz = 0;       // the setting the gains were computed for
static volatile uint8_t if_ready = 0;

/// @brief The position input filter starts again from the shaft (the control interrupt calls this whenever the
/// position and PD modes are not running).
void Input_filter_reset()
{
  if_ready = 0;
}

/// @brief The position input filter, parameter 74 (Hz): a critically
/// damped second-order filter from the host's position and speed setpoints to the loops, so the staircase a 50-250 Hz
/// host sends becomes a smooth path. Position mode (speed = the feedforward) and PD mode (speed = the setpoint) only.
/// When it runs, it replaces the position error and the speed the mode uses; otherwise it leaves them as they are.
void Input_filter(int32_t pos_in, float vel_in, float *position_error, float *speed)
{
  const int mode = controller.Controller_mode;
  if (controller.input_filter_hz == 0 || (mode != 1 && mode != 4))
  {
    if_ready = 0;
    return;
  }
  if (controller.input_filter_hz != if_hz)
  {
    if_hz = controller.input_filter_hz;
    const float w = PI2 * (float)if_hz;
    if_ki = 2.0f * w;
    if_kp = w * w;
  }
  if (!if_ready)
  {
    if_pos_q = (int64_t)controller.Position_Ticks * 65536;
    if_vel = (float)controller.Velocity_Filter;
    if_ready = 1;
  }
  const float dp = (float)((int64_t)pos_in * 65536 - if_pos_q) * (1.0f / 65536.0f);
  const float accel = if_kp * dp + if_ki * (vel_in - if_vel);
  if_vel += accel * (float)LOOP_TIME;
  if_pos_q += (int64_t)(if_vel * ((float)LOOP_TIME * 65536.0f));
  *position_error = (float)(if_pos_q - (int64_t)controller.Position_Ticks * 65536) * (1.0f / 65536.0f);
  *speed = if_vel;
}

/// The current limit in force when #Cal started (batch 22d): the calibration sets its own 1600 mA for its steps;
/// it is put back when the calibration ends, so neither the runs after it nor a #Save use 1600 on a smaller motor.
float Cal_saved_current_limit = 0.0f;

void Cal_restore_current_limit()
{
  if (Cal_saved_current_limit > 0.0f)
    PID.Iq_current_limit = Cal_saved_current_limit;
  Cal_saved_current_limit = 0.0f;
}

/// @brief This board's ID (parameter 86): the chip's factory-unique 96 bits folded into 32 (FNV-1a). Never changes.
uint32_t Board_id()
{
  const uint8_t *uid = (const uint8_t *)UID_BASE;
  uint32_t h = 2166136261u;
  for (int i = 0; i < 12; i++)
    h = (h ^ uid[i]) * 16777619u;
  return h;
}

/// @brief A new calibration ID (parameter 87), at every successful calibration; #Save keeps it. This chip has no random
/// number generator, so it mixes what differs from run to run (the time the calibration took, the timer, the KV it
/// measured) with the previous ID and the board ID. 0 is kept for "none yet".
void Calibration_id_new()
{
  uint32_t kv = 0;
  const float measured = controller.KV;
  memcpy(&kv, &measured, 4);
  const uint32_t mix[] = {controller.calibration_id, Board_id(), micros(), SysTick->VAL, kv};
  uint32_t h = 2166136261u;
  for (uint32_t m : mix)
    for (int b = 0; b < 32; b += 8)
      h = (h ^ ((m >> b) & 255u)) * 16777619u;
  controller.calibration_id = h != 0 ? h : 1;
}

/// Stack margin (batch 23, parameter 83): setup() paints the free RAM between the heap and the stack at its end; the
/// bytes from the bottom that still hold the pattern were never used by the stack (or later heap) since then.
extern "C" void *_sbrk(int incr);
static uint8_t *stack_paint_lo = nullptr;
static uint8_t *stack_paint_hi = nullptr;

void Stack_paint()
{
  uint8_t *lo = (uint8_t *)_sbrk(0);
  uint8_t *sp;
  __asm volatile("mov %0, sp" : "=r"(sp));
  uint8_t *hi = sp - 64; // leave the frames of this call alone
  if (lo == nullptr || hi <= lo)
    return;
  const uint32_t irq = __get_PRIMASK();
  __disable_irq(); // the control interrupt shares this stack: no frame of it may sit in the painted part
  for (uint8_t *p = lo; p < hi; p++)
    *p = 0xA5;
  __set_PRIMASK(irq);
  stack_paint_lo = lo;
  stack_paint_hi = hi;
}

int32_t Stack_unused()
{
  if (stack_paint_lo == nullptr)
    return -1;
  const uint8_t *p = stack_paint_lo;
  while (p < stack_paint_hi && *p == 0xA5)
    p++;
  return (int32_t)(p - stack_paint_lo);
}
