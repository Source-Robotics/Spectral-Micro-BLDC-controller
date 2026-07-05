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
#define FIRMWARE_VERSION 105

#define MAX_DRIVE_CURRENT 2800 // 2500 mA

#define MT6816_NO_MAGNET_BIT 0x0002

#define MAX_DUTY 0.95f

#define MIN_DUTY 0.05f

#define OVERMODULATION 1.154 // 1 no overmodulation, 1.154F SPWM overmodulation

#define ADC_RESOLUTION_ 12

#define MAX_TEMPERATURE 80

#define PI 3.14159265359f

#define PI2 6.283185307f

#define SQRT3DEV2 0.8660254038f

#define CPR 16384

#define RAD_CONST 0.000383495187f

#define CPR2 16384 / 2

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