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

  if (controller.DIR_ == 1)
  {
    controller.Sense1_Raw = ADC_CHANNEL_4_READ_SENSE1();
    controller.Sense2_Raw = ADC_CHANNEL_3_READ_SENSE2();
  }
  else
  {
    controller.Sense1_Raw = ADC_CHANNEL_3_READ_SENSE2();
    controller.Sense2_Raw = ADC_CHANNEL_4_READ_SENSE1();
  }

  controller.VBUS_RAW = ADC_CHANNEL_6_READ_VBUS();
  result1 = result1 << 8;
  digitalWriteFast(CSN, LOW);
  result2 = SPI.transfer16(0x8400);
  digitalWriteFast(CSN, HIGH);
  result = result1 | result2;
  controller.Position_Raw = result >> 2;
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
  /* Moving average filter on the velocity */
  controller.Velocity_Filter = movingAverage(controller.Velocity);
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
  controller.Position_Raw = result >> 2;
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
  if (controller.VBUS_mV < controller.Min_Vbus || controller.VBUS_mV > controller.Max_Vbus)
  // if (controller.VBUS_mV > controller.Max_Vbus)
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

  /// If there is no error driver can operate normaly
  /// If error is active we will immediately go to idle
  if (controller.Error == 0)
  {
    switch (controller.Controller_mode)
    {
    case 0:
      /// Sleep mode input.Logic high to enable device;
      /// logic low to enter low-power sleep mode; internal pulldown
      digitalWriteFast(SLEEP, LOW);
      /// Reset input. Active-low reset input initializes internal logic, clears faults,
      /// and disables the outputs, internal pulldown
      digitalWriteFast(RESET, LOW);
      controller.reset_pin_state = 0;
      controller.sleep_pin_state = 0;
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
    default:
      /// Idle
      break;
    }
  }
  else /// If there is error driver is disabled and in sleep mode
  {
    controller.Controller_mode = 0;
    // Sleep mode input.Logic high to enable device;
    // logic low to enter low-power sleep mode; internal pulldown
    digitalWriteFast(SLEEP, LOW);
    // Reset input. Active-low reset input initializes internal logic, clears faults,
    // and disables the outputs, internal pulldown
    digitalWriteFast(RESET, LOW);
    controller.reset_pin_state = 0;
    controller.sleep_pin_state = 0;
  }

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
    vel_accum += controller.Velocity_Filter;
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
    vel_accum += controller.Velocity_Filter;
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
    vel_accum += controller.Velocity_Filter;
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

  static int KV_ticks = 0;
  static int KV_duration = 5000;
  static int64_t KV_speed_accumulator = 0;
  static int64_t KV_current_accumulator = 0;

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
      controller.Sense1_Raw = ADC_CHANNEL_4_READ_SENSE1();
      controller.Sense2_Raw = ADC_CHANNEL_3_READ_SENSE2();
      controller.Sense1_mA = Get_current_mA(controller.Sense1_Raw);
      controller.Sense2_mA = Get_current_mA(controller.Sense2_Raw);
      int cur_avg = (abs(controller.Sense1_mA) + abs(controller.Sense1_mA)) / 2;
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
            controller.Resistance = controller.Resistance * (float)1.25; // + 2???
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
        controller.Sense1_Raw = ADC_CHANNEL_4_READ_SENSE1();
        controller.Sense2_Raw = ADC_CHANNEL_3_READ_SENSE2();
        controller.Sense1_mA = Get_current_mA(controller.Sense1_Raw);
        controller.Sense2_mA = Get_current_mA(controller.Sense2_Raw);
        int cur_avg = (abs(controller.Sense1_mA) + abs(controller.Sense1_mA)) / 2;
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
        PID.Ki = (float)1 - (float)exp(-controller.Resistance * LOOP_TIME / controller.Inductance);
        controller.Crossover_frequency = ((float)controller.current_control_bandwidth) / (float)(LOOP_FREQ / PI2);
        PID.Kp_iq = controller.Resistance * (float)(controller.Crossover_frequency / (float)(PID.Ki));
        PID.Ki_iq = (float)PID.Ki * (float)PID.Kp_iq;
        PID.Kp_id = PID.Kp_iq;
        PID.Ki_id = PID.Ki_iq;
        if (PID.Kp_iq < 0 || PID.Ki_iq < 0)
        {
          controller.Inductance_cal_status = 0;
          controller.Calib_error = 0;
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
      theta_SIM = fmod(theta_SIM, PI2);
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

      theta_SIM_command = fmod(theta_SIM, PI2);
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
      controller.pole_pairs = round((float)CPR / (abs((controller.Position_Ticks) - (start_position)) / controller.Number_of_rotations));
      // If there is error. Pole pair cant be 0 or negative and we limit upper limit to 100PP
      if (controller.pole_pairs <= 0 || controller.pole_pairs > 100)
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
          dq0_abc_variables(controller.Electric_Angle);
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
          dq0_abc_variables(controller.Electric_Angle);
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
            Phase_step = 3; // moved far enough in EITHER direction
          }
          else
          {
            // barely moved -> commutation genuinely not working
            PID.Uq_setpoint = 0;
            Voltage_Torque_mode();
            Phase_ticks = 0;
            Phase_start = 1;
            Phase_step = 0;
            controller.Kt_cal_status = 1;
            controller.Phase_order_status = 1;
            controller.Calib_error = 1;
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

  /// Calculate Kt, KV and Flux linkage
  if (Phase_order_step == 1 && KV_step == 0 && controller.Calib_error == 0)
  {

    if (KV_ticks <= KV_duration)
    {
      PID.Iq_current_limit = 1600;
      PID.Iq_setpoint = 1250;
      Collect_data();
      Torque_mode();
      dq0_abc_variables(controller.Electric_Angle);
      dq0_fast_int(controller.Sense1_mA, controller.Sense2_mA, controller.Sense3_mA, &FOC.Id, &FOC.Iq);
      KV_ticks = KV_ticks + 1;
      float rpm_speed = (controller.Velocity_Filter * 60) / 16384;
      KV_speed_accumulator = KV_speed_accumulator + rpm_speed;
      KV_current_accumulator = KV_current_accumulator + FOC.Iq;
    }
    else
    {

      float rpm_filtered = KV_speed_accumulator / KV_duration;
      float current_filtered = KV_current_accumulator / KV_duration;
      int load_correction = PID.Iq_setpoint / (PID.Iq_setpoint - current_filtered);
      float KV_value = (rpm_filtered / (controller.VBUS_mV / 1000)) * load_correction;
      controller.KV = KV_value;
      controller.Kt = float(8.27) / controller.KV;
      controller.flux_linkage = (float)2 / 3 * (float)controller.Kt / (float)controller.pole_pairs;

      controller.temp1 = KV_value;
      pwm_set(PWM_CH1, 0, 13);
      pwm_set(PWM_CH2, 0, 13);
      pwm_set(PWM_CH3, 0, 13);
      KV_step = 1;
      controller.KV_status = 2;
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
  }

  // If there is error
  if (controller.Calib_error == 1)
  {
    // Detach calibration interrupt routine
    Ticker_detach(TIM3);
    controller.Calibration = 2;
    controller.Calibrated = 0;
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
  }
}

/// @brief Spin the motor in the open loop
/// @param speed Speed in RAD/s; NOTE this is electrical speed of the motor! Speed_mech = speed_el / Pole_pairs
/// @param voltage Voltage in milivolts
void Open_loop_speed(float speed_el, int voltage)
{

  static float theta_SIM = 0;
  theta_SIM += speed_el * LOOP_TIME;
  theta_SIM = fmod(theta_SIM, PI2);
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
      if (controller.Inductance_cal_status == 1)
      {
        Serialport.print("Open loop: Failed");
        Serialport.println(" ");
      }
      else if (controller.Inductance_cal_status == 2)
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
      Serialport.print("Save config with #Save");
      Serialport.println(" ");
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

/// @todo feedforwards
/// @brief FOC cascaded position mode
/// @todo add cogging comp
void Position_mode()
{
  /* Positon PIDS*/
  float position_error = PID.Position_setpoint - controller.Position_Ticks;
  PID.P_errSum = PID.Kp_p * position_error;
  PID.Velocity_setpoint = PID.P_errSum + PID.Feedforward_speed;

  /*Clamp velocity to velocity limit*/
  if (PID.Velocity_setpoint > PID.Velocity_limit)
    PID.Velocity_setpoint = PID.Velocity_limit;
  else if (PID.Velocity_setpoint < -PID.Velocity_limit)
    PID.Velocity_setpoint = -PID.Velocity_limit;

  /* Velocity PIDS*/
  float velocity_error = PID.Velocity_setpoint - controller.Velocity_Filter;
  PID.V_errSum = PID.V_errSum + velocity_error * PID.Ki_v;

  /* Clamp integral term*/
  if (PID.V_errSum > PID.Iq_current_limit)
    PID.V_errSum = PID.Iq_current_limit;
  else if (PID.V_errSum < -PID.Iq_current_limit)
    PID.V_errSum = -PID.Iq_current_limit;

  /* Calculate Iq setpoint*/
  PID.Iq_setpoint = PID.Kp_v * velocity_error + PID.V_errSum + PID.Feedforward_current; // + cogging_comp

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
  PID.V_errSum = PID.V_errSum + velocity_error * PID.Ki_v;

  /* Clamp integral term*/
  if (PID.V_errSum > PID.Iq_current_limit)
    PID.V_errSum = PID.Iq_current_limit;
  else if (PID.V_errSum < -PID.Iq_current_limit)
    PID.V_errSum = -PID.Iq_current_limit;

  /* Calculate Iq setpoint*/
  PID.Iq_setpoint = PID.Kp_v * velocity_error + PID.V_errSum + PID.Feedforward_current; // + cogging_comp

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
  PID.Iq_setpoint = PID.KP * (PID.Position_setpoint - controller.Position_Ticks) + PID.KD * (PID.Velocity_setpoint - controller.Velocity_Filter) + PID.Feedforward_current;
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
  if (controller.DIR_ == 1)
  {
    controller.Sense1_Raw = ADC_CHANNEL_4_READ_SENSE1();
    controller.Sense2_Raw = ADC_CHANNEL_3_READ_SENSE2();
  }
  else
  {
    controller.Sense1_Raw = ADC_CHANNEL_3_READ_SENSE2();
    controller.Sense2_Raw = ADC_CHANNEL_4_READ_SENSE1();
  }

  controller.VBUS_RAW = ADC_CHANNEL_6_READ_VBUS();
  result1 = result1 << 8;
  digitalWriteFast(CSN, LOW);
  result2 = SPI.transfer16(0x8400);
  digitalWriteFast(CSN, HIGH);
  result = result1 | result2;
  controller.Position_Raw = result >> 2;
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
  /* Moving average filter on the velocity */
  controller.Velocity_Filter = movingAverage(controller.Velocity);
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

#define POSITION_TOLERANCE 5
#define VELOCITY_CONTACT_THRESHOLD 500.0f
#define CURRENT_CONTACT_RATIO 0.8f

/// @brief Gripper mode
void Gripper_mode()
{
  // --- Reset position flag if a new command was received ---
  if (Gripper.Same_command == 0)
  {
    Gripper.At_position = 0; // Only reset here when new command arrives
  }

  PID.Iq_current_limit = Gripper.current_setpoint;
  PID.Position_setpoint = map(Gripper.position_setpoint, 0, 255, Gripper.max_open_position, Gripper.max_closed_position);
  int vel_setpoint = map(Gripper.speed_setpoint, 0, 255, Gripper.min_speed, Gripper.max_speed);

  if (PID.Position_setpoint > controller.Position_Ticks)
    PID.Velocity_setpoint = vel_setpoint;
  else if (PID.Position_setpoint < controller.Position_Ticks)
    PID.Velocity_setpoint = -vel_setpoint;

  if (Gripper.action_status == 1 && Gripper.calibrated && Gripper.activated && controller.I_AM_GRIPPER)
  {
    int pos_error = abs(PID.Position_setpoint - controller.Position_Ticks);

    // ✅ If within tolerance, mark as at position (and never unset again)
    if (pos_error < POSITION_TOLERANCE)
    {
      Gripper.At_position = 1;
    }

    if (Gripper.At_position)
    {
      // ✅ Once reached, stay in position mode — never go back to velocity
      Position_mode();
      Gripper.object_detection_status = 3; // at position
    }
    else
    {
      // ✅ Only used while still approaching the target
      Velocity_mode();

      float current_abs = fabs(FOC.Iq);
      float velocity_abs = fabs(controller.Velocity_Filter);

      if (velocity_abs > VELOCITY_CONTACT_THRESHOLD)
      {
        Gripper.object_detection_status = 0; // still moving freely
      }
      else if (current_abs > CURRENT_CONTACT_RATIO * PID.Iq_current_limit)
      {
        Gripper.object_detection_status = (FOC.Iq > 0) ? 2 : 1;
      }
      else
      {
        Gripper.object_detection_status = 0;
      }
    }
  }
  else
  {
    // --- Idle or disabled ---
    PID.Velocity_setpoint = 0;
    Velocity_mode();

    // Sleep mode input.Logic high to enable device;
    // logic low to enter low-power sleep mode; internal pulldown
    digitalWriteFast(SLEEP, LOW);
    // Reset input. Active-low reset input initializes internal logic, clears faults,
    // and disables the outputs, internal pulldown
    digitalWriteFast(RESET, LOW);
    controller.reset_pin_state = 0;
    controller.sleep_pin_state = 0;

    float current_abs = fabs(FOC.Iq);
    float velocity_abs = fabs(controller.Velocity_Filter);

    if (velocity_abs < VELOCITY_CONTACT_THRESHOLD && current_abs > CURRENT_CONTACT_RATIO * PID.Iq_current_limit)
    {
      Gripper.object_detection_status = (FOC.Iq > 0) ? 2 : 1;
    }
    else
    {
      Gripper.object_detection_status = 0;
    }
  }

  Gripper.Same_command = 1;
}

/// @brief We need to calibrate the gripper before it can be used
/// Calibration moves gripper to fully open position; maps the value
/// Move to the fully closed position and map the value. Positions are detected
/// If large enough current is detected. After this is complete set Gripper.calibrated = 1
void Calibrate_gripper()
{

  static int tick_gripper_cnt = 0;
  static bool grip_cal_1 = 0;
  static bool grip_cal_2 = 0;
  static int grip_delay_1 = 0;
  static int grip_delay_tick_1 = 0;

  static int open_confirm_cnt = 0;
  static int close_confirm_cnt = 0;

  int current_limit = 500;
  int speed_limit = 40; // Lower this if it does not home
  Gripper.In_calibration = 1;

  tick_gripper_cnt = tick_gripper_cnt + 1;

  /// Fully close the gripper (This is step two)
  if (grip_cal_1 == 1 && grip_cal_2 == 0 && grip_delay_1 == 1)
  {
    /// Transform setpoints into motor redable params
    PID.Iq_current_limit = current_limit;
    /// PID.Position_setpoint = map(Gripper.position_setpoint, 0, 255, Gripper.max_open_position, Gripper.max_closed_position);
    int vel_sepoint = map(speed_limit, 0, 255, Gripper.min_speed, Gripper.max_speed);
    PID.Velocity_setpoint = -vel_sepoint;
    Velocity_mode();

    /// If it is not moving and current is around the setpoint

    if (isAroundValue(abs(controller.Velocity_Filter), 0, 700) &&
        isAroundValue(abs(FOC.Iq), PID.Iq_current_limit, 90))
    {
      close_confirm_cnt++;
    }
    else
    {
      close_confirm_cnt = 0;
    }

    if (close_confirm_cnt >= 3200)
    {
      Gripper.max_closed_position = controller.Position_Ticks;
      grip_cal_2 = 1;
      close_confirm_cnt = 0;
    }
  }
  /***********************************/

  /// Fully open the gripper (This is step one)
  if (grip_cal_1 == 0)
  {
    /// Transform setpoints into motor redable params
    PID.Iq_current_limit = current_limit;
    /// PID.Position_setpoint = map(Gripper.position_setpoint, 0, 255, Gripper.max_open_position, Gripper.max_closed_position);
    int vel_sepoint = map(speed_limit, 0, 255, Gripper.min_speed, Gripper.max_speed);
    PID.Velocity_setpoint = vel_sepoint;
    Velocity_mode();
    /// If it is not moving and current is around the setpoint

    if (isAroundValue(abs(controller.Velocity_Filter), 0, 700) &&
        isAroundValue(abs(FOC.Iq), PID.Iq_current_limit, 90))
    {
      open_confirm_cnt++;
    }
    else
    {
      open_confirm_cnt = 0;
    }

    if (open_confirm_cnt >= 3200)
    {
      Gripper.max_open_position = controller.Position_Ticks;
      grip_cal_1 = 1;
      open_confirm_cnt = 0;
    }
  }
  /***********************************/

  /// Small delay between direction change
  if (grip_cal_1 == 1 && grip_delay_1 == 0)
  {
    // digitalWriteFast(SLEEP, LOW);
    // digitalWriteFast(RESET, LOW);
    controller.reset_pin_state = 0;
    controller.sleep_pin_state = 0;
    grip_delay_tick_1 = grip_delay_tick_1 + 1;
    PID.Velocity_setpoint = 0;
    Velocity_mode();
    if (grip_delay_tick_1 == 100)
    {
      grip_delay_tick_1 = 0;
      grip_delay_1 = 1;
    }
  }
  /***********************************/

  /// At the end reset everything
  if (grip_cal_1 == 1 && grip_cal_2 == 1 && grip_delay_1 == 1)
  {

    Gripper.calibrated = 1;

    grip_cal_1 = 0;
    grip_cal_2 = 0;

    grip_delay_1 = 0;
    grip_delay_tick_1 = 0;
    tick_gripper_cnt = 0;
    // digitalWriteFast(SLEEP, LOW);
    // digitalWriteFast(RESET, LOW);
    controller.reset_pin_state = 0;
    controller.sleep_pin_state = 0;
    controller.Controller_mode = 0;
    Gripper.In_calibration = 0;
    open_confirm_cnt = 0;
    close_confirm_cnt = 0;
  }
  /***********************************/

  /// Calibration timed out
  /// during whole calibration we keep track of tick_gripper_cnt
  /// One tick is equal to interrupt time.
  if (tick_gripper_cnt >= 150000)
  {
    Gripper.calibrated = 0;

    grip_cal_1 = 0;
    grip_cal_2 = 0;

    grip_delay_1 = 0;
    grip_delay_tick_1 = 0;
    tick_gripper_cnt = 0;
    // digitalWriteFast(SLEEP, LOW);
    // digitalWriteFast(RESET, LOW);
    controller.reset_pin_state = 0;
    controller.sleep_pin_state = 0;
    controller.Controller_mode = 0;
    Gripper.In_calibration = 0;
    open_confirm_cnt = 0;
    close_confirm_cnt = 0;
  }
}