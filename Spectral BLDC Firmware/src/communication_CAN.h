/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    communication_CAN.h
  * @brief   This file provides code for all the function prototypes for communication_CAN.cpp
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


#ifndef CAN_COMS_H
#define CAN_COMS_H

#include "constants.h"
#include <stdint.h>
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
#include <SPI.h>
#include "common.h"
#include "foc.h"
#include "temperature_table.h"
#include "CAN.h"
#include <Wire.h>
#include <I2C_eeprom.h>
#include "CAN.h"
#include "EEPROM.h"

void packData1(int position, int speed, int torque, byte *bytes);

void Setup_CAN_bus();

void CAN_protocol(Stream &Serialport);
  
void Cyclic_CAN(uint32_t ms);

void CAN_test(Stream &Serialport);    

void Ping_response();

void Extract_from_CAN_ID(unsigned int canId);

unsigned int Combine_2_CAN_ID(unsigned int ID, unsigned int canCommand, bool errorBit);

void Heartbeat_CAN();

void CAN_heartbeat(uint32_t ms);

void CAN_watchdog(uint32_t ms);

void Temperature_CAN();

void Voltage_CAN();

void Device_info_CAN();

bool Param_get(uint8_t param, int32_t *value);

bool Param_set(uint8_t param, int32_t value);
bool Param_is_float(uint8_t param); // travels as a float's bits

void Param_value_CAN(uint8_t param);

void Identity_CAN();

void Telemetry_CAN();

void Encoder_data_CAN();

void Current_data_CAN();

void State_of_Errors_CAN();

void Data_pack_1_CAN();

void Data_pack_2_CAN();

void Data_pack_3_CAN();

void Gripper_pack_data();

void KT_data_CAN();


#endif