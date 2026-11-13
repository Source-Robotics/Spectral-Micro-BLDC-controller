/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_control.h
  * @brief   This file provides code for all the function prototypes for motor_control.cpp
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
#ifndef MOTOR_CONT_
#define MOTOR_CONT_

#include <Arduino.h>
#include <stdio.h>
#include "hw_init.h"
#include "pwm_init.h"
#include "utils.h"
#include "stm32f1xx_hal.h"
#include "adc_init.h"
#include "iodefs.h"
#include "qfplib-m3.h"
#include "SerialPacketParser.h"
#include "constants.h"
#include "motor_control.h"
#include "common.h"
#include "temperature_table.h"


const float CURRENT_SENSE_CONSTANT = ((ADC_MIDPOINT / ADC_MIDPOINT_BIT) / SENSE_RESISTOR) / CURRENT_AMP_GAIN;
const float CURRENT_SENSE_CONSTANT_mV = ((ADC_MIDPOINT / ADC_MIDPOINT_BIT) / SENSE_RESISTOR) / CURRENT_AMP_GAIN * 1000;
//const int VOLTAGE_SENSE_CONSTANT = 

bool parityCheck(uint16_t data);
void Collect_data();
float Get_current(int adc_value);
int Get_voltage_mV(int adc_value);
int Get_current_mA(int adc_value);
int Get_ADC_Value(int current_mA);
void Position_mode();
void Velocity_mode();
void Torque_mode();
void Update_IT_callback_calib();
void IT_callback(void);
void Sense_zero_calibrate();
void Enable_drive(void);
void Calib_report(Stream &Serialport);
void PD_mode();
void Open_loop_speed(float speed, int voltage);
void LED_status(uint32_t ms);
void Phase_order();
void Collect_data2();
void Gripper_mode();
void Calibrate_gripper();
void Gripper_calibration_abort();
void Get_first_encoder();
void Voltage_Torque_mode();
int Calibrate_Angle_Offset2();
int Calibrate_Angle_Offset_Align();
void Motion_wake(); // what a motion frame does to the bridge before it moves (cogcal.cpp)
bool Current_gains_from_RL();       // current PI gains from R, L and current_control_bandwidth
float Current_bandwidth_in_force(); // the bandwidth, Hz, the current gains in force give
int Speed_estimate(int velocity_raw, int d_ticks); // the speed the loops use (parameter 50 picks it)
void Speed_pll_gains();            // the speed PLL's gains from speed_pll_bandwidth
bool Sense_trim_from_h1(float a, float b, int32_t *d1, int32_t *d2); // zero errors behind a 1st harmonic (cogcal)
void Comm_delay_update();          // the commutation delay rotation for this loop (parameter 64)
float Velocity_pi(float err_p, float err_i); // the velocity PI, clamped (batch 18)
float Vel_int_setpoint(float position_error, float speed_ff); // the integral's speed setpoint, deadband 75
void Input_filter(int32_t pos_in, float vel_in, float *position_error, float *speed); // parameter 74
void Input_filter_reset();
extern float Cal_saved_current_limit; // the current limit when #Cal started (batch 22d)
void Cal_restore_current_limit();     // put it back when the calibration ends
void Stack_paint();                   // end of setup(): the free RAM painted (parameter 83)
extern volatile int32_t Sense_sample_at;   // parameter 88: PWM count to read the currents at; -1 as before
extern volatile int32_t Sense_sample_seen; // parameter 89: where the last reading happened
int32_t Stack_unused();               // bytes of it the stack never reached
uint32_t Board_id();                  // parameter 86: the chip's unique number in 32 bits
void Calibration_id_new();            // parameter 87: a new one, at a successful calibration
void Foc_frame(float theta);       // Id, Iq and the inverse transform at this electrical angle (encal.cpp)

#endif
