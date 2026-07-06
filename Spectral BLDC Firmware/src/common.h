
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    common.h
 * @brief   This file provides code where we declare global structures
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

#ifndef COMMON_H
#define COMMON_H

#include "constants.h"
#include <stdint.h>

/// @brief  Structure for general controller variables
typedef struct
{

    volatile int SERIAL_NUMBER = 1;
    volatile int HARDWARE_VERSION = 1; // Version of the Spectral BLDC driver PCB
    volatile int BATCH_DATE = 1;       // Date the batch was produced
    volatile int SOFTWARE_VERSION = 1; // Software release
    volatile bool I_AM_GRIPPER = 1;    // Is motor controller gripper (1) or not (0)
    volatile int LED_ON_OFF = 1;       // Use LED for status indication

    volatile uint32_t interrupt_tick = 0; // Ticks of our interrupt routine

    volatile int Sense1_Raw; // RAW ADC measure of current 1
    volatile int Sense2_Raw; // RAW ADC measure of current 2
    volatile int VBUS_RAW;   // RAW ADC measure of VBUS
    volatile int TEMP_RAW;   // RAW ADC measure of temperature

    volatile int Sense1_mA;     // Current 1 in mA range
    volatile int Sense2_mA;     // Current 2 in mA range
    volatile int Sense3_mA;     // Current 3 in mA range (calculated from current 1 and 2)
    volatile int VBUS_mV;       // Vbus in mV range
    volatile int TEMP_DEG = 23; // Temperature in degrees celsius

    volatile uint16_t Position_Raw;                 // RAW position from sensor 0 - 16383
    volatile uint16_t Old_Position_Raw;             // RAW position from sensor from previous cycle
    volatile int Position_Ticks;                    // Position with multiple rotations
    volatile int Old_Position_Ticks;                // position with multiple rotations from previous cycle
    volatile float Position_RAD;                    // Raw position but in radians
    volatile float Electric_Angle;                  // Electrical angle - depends on number of pole pairs
    volatile float theta_offset = 0.0f; // will be determined at calibration

    volatile int ROTATIONS;                         // Number of full rotations that motor did
    volatile int Velocity_Filter;                   // Filtered velocity of the motor
    volatile int Velocity;                          //  velocity of the motor
    volatile float Torque_estimate;                 // Kt * Iq
    volatile int pole_pairs = 11;                   // Number of pole pairs of your BLDC motor
    volatile float current_control_bandwidth = 500; // default 200 HZ; Will set PI gains for Q and D current loops based on phase resistance
                                                    // phase inductance and loop time
                                                    // RAD/s = HZ * 2pi

    volatile int sleep_pin_state = 0;
    volatile int reset_pin_state = 0;

    volatile float Crossover_frequency = 0.0; // Crossover frequency, in Radians per sample

    volatile int Magnet_warrning; // 0 is for detected magnet, 1 is no magnet or not detecting enough flux density
    volatile int MT6816_parity_check_pass;
    volatile int DIR_ = 1; // When looking at the magnetic encoder counter-clockwise is POSITIVE rotation.
    volatile int Phase_order = 2;
    volatile int commutation_dir = 1; // +1 or -1, auto-detected at calibration so +Iq/+velocity/+rotation map to +encoder rotation

    volatile int Thermistor_on_off = 0; // Are we taking termistor mesurements? 0 is off, 1 is on

    volatile int Controller_mode = 0; // 0 -> idle, 1 -> Positon, 2 -> Speed, 3 -> Current, 4 -> PD, 5 -> Open-loop speed, 6 -> Gripper mode, 7-> Calibrate gripper, 8 -> Voltage Torque mode
    // 7 -> Gripper calib mode

    volatile float Resistance = 0;       // Resistance of SINGLE PHASE of your BLDC motor
    volatile float Total_Resistance = 0; // = 2 * Resistance + 2 * R_sense + 2 *Rdson
    volatile float Inductance = 0;       // Inductance of SINGLE PHASE of your BLDC motor
    volatile float Kt = 0;               // Torque constant of your BLDC motor
    volatile float flux_linkage = 0;     // Flux linkage of your BLDC motor
    volatile float KV = 0;

    volatile int Max_temperature = 75;  // Max temperature in degrees
    volatile int Min_temperature = -20; // Min temperature in degrees

    volatile int Max_Vbus = 29500; // Max vbus voltage in mV
    volatile int Min_Vbus = 9000;  // Min vbus voltage in mV

    volatile int execution_time = 0; // Execution time of our interrupt loops

    volatile int watchdog_time_ms = 0;
    volatile bool watchdog_reset = 1;
    volatile int watchdog_action = 0;   // Hold position, Idle, brake
    volatile int Heartbeat_rate_ms = 0; // Motor controller will send data every Heartbeat_rate_ms
    volatile bool Send_heartbeat = 0;

    volatile bool Activated = 0;         // If controller is activated or not
    volatile bool Error = 0;             // General error
    volatile bool temperature_error = 0; // Termistor reported error
    volatile bool encoder_error = 0;     // Encoder magnet is not alligned error
    volatile bool Vbus_error = 0;        // Vbus voltage is too small or too large
    volatile bool Driver_error = 0;      // Driver reported error. 3 possible conditions: Undervoltage, thermal shutdown, overcurrent
    volatile bool Velocity_error = 0;    // We are moving too fast
    volatile bool Current_error = 0;     // We go past max current
    volatile bool ESTOP_error = 0;       // If we received estop command
    volatile bool Watchdog_error = 0;    // If we pass watchdog timeout time
    volatile bool CAN_init_error = 0;    // Set if CAN peripheral failed to initialize; CAN bus is skipped, UART stays usable

    volatile int commutation = 0; // 0 is space vector, 1 is sine

    volatile int Calibration = 0;                    // 0 - outside of calibration, 1 inside of calibration, 2 failed calibration, 3 success calibration
    volatile bool Calibrated = 0;                    // 0 if motor is not calibrated, 1 if it is calibrated
    volatile int Resistance_calc_voltage = 1000;     // Voltage used to calculate resistance //1000
    volatile float Calibration_power = 12;           // Max power that can dissipate during inductance test.
    volatile int Calibration_Inductance_voltage = 0; // Calculated from Calibration power and resistance
    volatile int Open_loop_voltage = 5000;           // Voltage to use when spinning the motor open loop. In mV
    volatile float Open_loop_speed = 20.0;           // Speed to be used when spinning the motor open loop. In RAD/s
    volatile int Number_of_rotations = 8;            // Number of open loop electrical rotations during calibration
    volatile int Phase_voltage = 5000;               // Max voltage to be used during Kv calculation

    // Calibration steps
    // 0 is neutral, 1 is error, 2 is success
    volatile int Calib_error = 0;
    volatile int Magnet_cal_status = 0;
    volatile int Vbus_cal_status = 0;
    volatile int Resistance_cal_status = 0;
    volatile int Inductance_cal_status = 0;
    volatile int Open_loop_cal_status = 0;
    volatile int Kt_cal_status = 0;
    volatile int Phase_order_status = 0;
    volatile int KV_status = 0;

    // Cyclic UART
    volatile int cyclic = 0;        // If 1 we Periodically send messages without request from host.
    volatile int cyclic_a = 0;      // If 1 we Periodically send messages without request from host.
    volatile int cyclic_period = 0; // We send messages every cyclic_period

    // CAN bus
    // CAN ID is 4 bit variable. so it can be:
    // 0-15 in decimal system (DEC)
    // 0x0 - 0xF in hexadecimal system (HEX)
    volatile int CAN_ID = 0; // CAN id of THIS motor driver! No 2 motor drivers can share the same ID.
    volatile bool Wrong_DL = 0;

    // temporary test variables
    volatile float temp = 0;
    volatile float temp1 = 0;
    volatile float temp2 = 0;

    volatile bool handle_can = 0;

    volatile bool hall_trigger = 1; // used to record if hall sensor was triggered or not, normal state of hall is high (1)
    volatile bool trigger_value = 0;

    // temp / testing ?
    volatile int calibration_offset_current = 300;
    volatile int temp_var_offset_calib = 0;
    volatile int align_state = 0;
    volatile int align_counter = 0;
    volatile float aligned_angle = 0;
    volatile bool align_done = false;
    volatile bool calib_done = false;
    volatile bool calib_reset = false;
    volatile int Velocity_fwd = 0;
    volatile int Velocity_bwd = 0;
    volatile int Align_stage = 0; // Calibrate_Angle_Offset_Align() progress: state index (0 = not running)
    volatile int Align_point = 0; // sweep index within the current stage

    // Open loop movements
    volatile int microstep = 16;
    volatile int gosteps = 0;
    volatile int step_interval = 1000;    // step interval in microseconds
    volatile int open_loop_move_done = 0; // 1 = finished, cleared by user

} Measure;

extern Measure controller;

/// @brief Structure for FOC variables
typedef struct
{

    volatile float Iq;
    volatile float Id;

    volatile float Uq;
    volatile float Ud;

    volatile float U1;
    volatile float U2;
    volatile float U3;

    volatile float U1_normalized;
    volatile float U2_normalized;
    volatile float U3_normalized;

    volatile float sine_value;   // value of sine
    volatile float cosine_value; // value of cosine

    volatile float sin_sqrt3div2; // 0.86602540378f * sine_value
    volatile float cos_sqrt3div2; // 0.86602540378f * cosine_value

    volatile float sin_05; // 0.5 * sin_value
    volatile float cos_05; // 0.5 * cos_value

    // sin_sqrt3div2 - cos_05
    volatile float const1; // 0.86602540378f * sine_value - 0.5 * cos_value

    // cos_sqrt3div2 - sin_05
    volatile float const2; // 0.86602540378f * cosine_value - 0.5 * sin_value

    // -sin_sqrt3div2 - cos_05
    volatile float const3; // 0.86602540378f * sine_value - 0.5 * cos_value

    // -cos_sqrt3div2 - sin_05
    volatile float const4; // 0.86602540378f * cosine_value - 0.5 * sin_value

    volatile int PWM1;
    volatile int PWM2;
    volatile int PWM3;

} _FOC;

extern _FOC FOC;

/// @brief  Structure for PID variables
typedef struct
{

    volatile int Voltage_limit = 0; // Voltage limit for Ud and Uq voltages. If 0 value of controller.VBUS_mV will be used
    // if != 0 it will use that number (if bigger than controller.VBUS_mV it will use controller.VBUS_mV)
    // This is extremely important for proper operation of BLDC motor loops since their resistances can be less
    // than 1 ohm and using full range of 24V makes the loop unstable.
    // Usually it setting this to 0 is good (so using full controller.VBUS_mV range)
    // Use smaller numbers if you have really small phase resistances like 1 ohm or less

    // Do we want to reset integral accumulators after receiving new setopint, needs to be 1 for the gripper
    volatile int Reset_integral_accumulator = 0;

    // Position loop PID
    // NOTE: these initializers must match Set_Default_config() (EEPROM.cpp) exactly --
    // that function is authoritative (what a #Default/factory-reset board actually gets).
    volatile float Kp_p = 11;
    volatile int Position_setpoint = 0;
    volatile float P_errSum = 0;

    // Speed loop PID
    volatile float Kp_v = 0.03;   //   0.006 basic values that work for most motors but not optimal
    volatile float Ki_v = 0.0003; //   0.0001 basic values that work for most motors but not optimal
    volatile float V_errSum = 0;
    volatile float Feedforward_speed = 0;
    volatile float Velocity_setpoint = 0; // -1.5

    volatile float Velocity_limit = 800000;         // Clamp integrals to this  [TICKS/s]
    volatile float Velocity_limit_error = 20000000; // [TICKS/s] ; Velocity when we will report error

    // current loop PID
    volatile float Ki = 0; // zero location

    volatile float Kp_id = 3;
    volatile float Ki_id = 1.5;
    volatile float Id_errSum;
    volatile float Id_setpoint = 0;

    volatile float Kp_iq = 3;
    volatile float Ki_iq = 1.5;
    volatile float Iq_errSum;
    volatile float Iq_setpoint;

    volatile float Iq_current_limit = 1000;
    volatile float Id_current_limit = 0;

    volatile float Feedforward_current = 0;

    // PD loop (Impedance controller)
    // https://en.wikipedia.org/wiki/Impedance_control
    volatile float KD = 0.002800; //  [Nm*s/rad]  KD 0.002800
    volatile float KP = 0.14000;  //  [Nm/rad] KP 0.14000

    volatile int Uq_setpoint = 0;
    volatile int Ud_setpoint = 0;

} PID_par;

extern PID_par PID;

typedef struct
{

    volatile uint8_t previousBytes[5] = {0, 0, 0, 0, 0};
    volatile bool Same_command = 0;
    volatile bool At_position = 0;

    volatile uint8_t position_setpoint = 0;
    volatile uint8_t speed_setpoint = 20;
    volatile int current_setpoint = 200;

    volatile uint8_t position = 0;
    volatile uint8_t speed = 0;
    volatile int current = 0;

    volatile bool calibrated = 0;
    volatile bool action_status = 0; // -0 Stopped(or performing activation /automatic release).
    volatile bool activated = 0;

    // 0 in motion, 1 object detected closing, 2 object detected opening, 3 at positon
    volatile int object_detection_status = 3;
    volatile bool temperature_error = 0;
    volatile bool timeout_error = 0;
    volatile bool estop_error = 0;

    volatile bool release_direction = 0;
    volatile bool estop_status = 0;

    // Map this to 255 and 0
    volatile int max_open_position = 10329;
    volatile int max_closed_position = -229;

    volatile int position_ticks = 0;

    // Map this to 255 and 0
    volatile int max_speed = 80000;
    volatile int min_speed = 40;

    volatile int In_calibration = 0;

} GRIPPER_STRUCT;

extern GRIPPER_STRUCT Gripper;

#endif
