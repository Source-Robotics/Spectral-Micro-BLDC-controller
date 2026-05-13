/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    common.cpp
 * @brief   This file provides code for utilities for EEPROM handling
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

#include "EEPROM.h"
#include "bootloader_config.h"

I2C_eeprom eeprom(DEVICEADDRESS, EEPROM);

/// @brief Write float value to pageadress location + 3.
/// @brief If page address is 0; data will be written to 0, 1,2 and 3
/// @param pageAddress page aaddress
/// @param data data we want to write
void writeFloat(unsigned int pageAddress, float data)
{
  uint8_t temp_byte[4] = {0, 0, 0, 0};
  memcpy(temp_byte, &data, 4);
  eeprom.writeByte(pageAddress, temp_byte[0]);
  eeprom.writeByte(pageAddress + 1, temp_byte[1]);
  eeprom.writeByte(pageAddress + 2, temp_byte[2]);
  eeprom.writeByte(pageAddress + 3, temp_byte[3]);
}

/// @brief Read float value from pageadress location + 3.
/// @brief If page address is 0; data will be read to 0, 1,2 and 3
/// @param pageAddress page aaddress
/// @return float value
float readFloat(unsigned int pageAddress)
{
  uint8_t temp_byte[4] = {0, 0, 0, 0};
  temp_byte[0] = eeprom.readByte(pageAddress);
  temp_byte[1] = eeprom.readByte(pageAddress + 1);
  temp_byte[2] = eeprom.readByte(pageAddress + 2);
  temp_byte[3] = eeprom.readByte(pageAddress + 3);
  float restoredFloat;
  memcpy(&restoredFloat, temp_byte, 4);
  return restoredFloat;
}

/// @brief Read int value from pageadress location + 3.
/// @brief If page address is 0; data will be read to 0, 1,2 and 3
/// @param pageAddress page aaddress
/// @return int value
int32_t readInt(unsigned int pageAddress)
{
  uint8_t temp_byte[4] = {0, 0, 0, 0};
  temp_byte[0] = eeprom.readByte(pageAddress);
  temp_byte[1] = eeprom.readByte(pageAddress + 1);
  temp_byte[2] = eeprom.readByte(pageAddress + 2);
  temp_byte[3] = eeprom.readByte(pageAddress + 3);
  int32_t restoredInt;
  memcpy(&restoredInt, temp_byte, 4);
  return restoredInt;
}

/// @brief Write int value to pageadress location + 3.
/// @brief If page address is 0; data will be written to 0, 1,2 and 3
/// @param pageAddress page aaddress
/// @param data data we want to write
void writeInt(unsigned int pageAddress, int32_t data)
{
  uint8_t temp_byte[4] = {0, 0, 0, 0};
  memcpy(temp_byte, &data, 4);
  eeprom.writeByte(pageAddress, temp_byte[0]);
  eeprom.writeByte(pageAddress + 1, temp_byte[1]);
  eeprom.writeByte(pageAddress + 2, temp_byte[2]);
  eeprom.writeByte(pageAddress + 3, temp_byte[3]);
}

/*
/// @brief Read byte value from pageadress location
/// @param pageAddress page aaddress
/// @return byte value
int8_t readInt_8t(unsigned int pageAddress)
{
  uint8_t temp_byte[4] = {0, 0, 0, 0};
  temp_byte[0] = eeprom.readByte(pageAddress);
  temp_byte[1] = eeprom.readByte(pageAddress + 1);
  temp_byte[2] = eeprom.readByte(pageAddress + 2);
  temp_byte[3] = eeprom.readByte(pageAddress + 3);
  int32_t restoredInt;
  memcpy(&restoredInt, temp_byte, 4);
  return restoredInt;
}

/// @brief Write int value to pageadress location
/// @param pageAddress page aaddress
/// @param data byte we want to write
void writeInt_8t(unsigned int pageAddress, int8_t data)
{
  uint8_t temp_byte[4] = {0, 0, 0, 0};
  memcpy(temp_byte, &data, 4);
  eeprom.writeByte(pageAddress, temp_byte[0]);
  eeprom.writeByte(pageAddress + 1, temp_byte[1]);
  eeprom.writeByte(pageAddress + 2, temp_byte[2]);
  eeprom.writeByte(pageAddress + 3, temp_byte[3]);
}
*/

/// @brief Init EEPROM memory
void Init_EEPROM()
{
  Wire.setSDA(EEPROM_SDA);
  Wire.setSCL(EEPROM_SCL);
  eeprom.begin();
  Wire.setClock(1000000);
}

/// @brief Read the config saved in the EEPROM
void read_config()
{

  controller.HARDWARE_VERSION = readInt(HARDWARE_VERSION_EEPROM);
  controller.BATCH_DATE = readInt(BATCH_DATA_EEPROM);
  controller.CAN_ID = readInt(CAN_ID_EEPROM);
  if (!spectral_can_id_int_is_valid(controller.CAN_ID)) {
    controller.CAN_ID = 0;
  }
  controller.SOFTWARE_VERSION = readInt(SOFTWARE_VERSION_EEPROM);
  controller.LED_ON_OFF = readInt(LED_ON_OFF_EEPROM);
  controller.Thermistor_on_off = readInt(THERMISTOR_ON_OFF_EEPROM);
  controller.pole_pairs = readInt(POLE_PAIR);
  controller.DIR_ = readInt(DIR_EEPROM);
  controller.Resistance = readFloat(RESISTANCE_EEPROM);
  controller.Total_Resistance = readFloat(TOTAL_RESISTANCE_EEPROM);
  controller.Inductance = readFloat(INDUCTANCE_EEPROM);

  controller.Kt = readFloat(KT_EEPROM);
  controller.KV = readFloat(KV_EEPROM);
  controller.flux_linkage = readFloat(FLUX_LINKAGE_EEPROM);
  PID.Kp_p = readFloat(KPP_EEPROM);
  PID.Kp_v = readFloat(KPV_EEPROM);
  PID.Ki_v = readFloat(KIV_EEPROM);

  PID.Velocity_limit = readFloat(VELOCITY_LIMIT_EEPROM);
  PID.Ki_iq = readFloat(KIIQ_EEPROM);
  PID.Kp_iq = readFloat(KPIQ_EEPROM);

  PID.Iq_current_limit = readInt(IQ_CURRENT_LIMIT_EEPROM);
  PID.Ki_id = readFloat(KIID_EEPROM);
  PID.Kp_id = readFloat(KPID_EEPROM);

  PID.Id_current_limit = readInt(ID_CURRENT_LIMIT_EEPROM);
  PID.KP = readFloat(KP_EEPROM);
  PID.KD = readFloat(KD_EEPROM);

  controller.Calibrated = readInt(CALIBRATED_EEPROM);
  controller.Phase_order = readInt(PHASE_ORDER_EEPROM);
  controller.watchdog_time_ms = readInt(WATCHDOG_TIME_EEPROM);
  controller.watchdog_action = readInt(WATCHDOG_ACTION_EEPROM);
  controller.Heartbeat_rate_ms = readInt(HEARTBEAT_RATE_EEPROM);

  controller.I_AM_GRIPPER = readInt(I_AM_GRIPPER_EEPROM);
  PID.Reset_integral_accumulator = readInt(RESET_INTEGRAL_EEPROM);

  controller.Max_temperature =   readInt(TEMPERATURE_ERROR);
  controller.Max_Vbus = readInt(VOLTAGE_ERROR);
  PID.Voltage_limit = readInt(VOLTAGE_LIMIT);
  controller.theta_offset = readFloat(THETA_OFFSET);

}

/// @brief Clean the config that is saved in the EEPROM
void Clean_config()
{
}

/// @brief Write latest data to the EEPROM
/// @details
void Write_config()
{

  // This will be removed!
  writeInt(SERIAL_NUMBER_EEPROM,controller.SERIAL_NUMBER);
  writeInt(HARDWARE_VERSION_EEPROM, controller.HARDWARE_VERSION);
  writeInt(BATCH_DATA_EEPROM, controller.BATCH_DATE);
  ///

  writeInt(CAN_ID_EEPROM, controller.CAN_ID);
  Bootloader_SyncBoardId((uint8_t)controller.CAN_ID);
  writeInt(SOFTWARE_VERSION_EEPROM, controller.SOFTWARE_VERSION);
  writeInt(LED_ON_OFF_EEPROM, controller.LED_ON_OFF);
  writeInt(THERMISTOR_ON_OFF_EEPROM, controller.Thermistor_on_off);
  writeInt(POLE_PAIR, controller.pole_pairs);
  writeInt(DIR_EEPROM, controller.DIR_);
  writeInt(PHASE_ORDER_EEPROM, controller.Phase_order);
  writeFloat(RESISTANCE_EEPROM, controller.Resistance);
  writeFloat(TOTAL_RESISTANCE_EEPROM, controller.Total_Resistance);
  writeFloat(INDUCTANCE_EEPROM, controller.Inductance);

  writeFloat(KT_EEPROM, controller.Kt);
  writeFloat(KV_EEPROM, controller.KV);
  writeFloat(FLUX_LINKAGE_EEPROM, controller.flux_linkage);
  writeFloat(KPP_EEPROM, PID.Kp_p);
  writeFloat(KPV_EEPROM, PID.Kp_v);
  writeFloat(KIV_EEPROM, PID.Ki_v);

  writeFloat(VELOCITY_LIMIT_EEPROM, PID.Velocity_limit);
  writeFloat(KIIQ_EEPROM, PID.Ki_iq);
  writeFloat(KPIQ_EEPROM, PID.Kp_iq);

  writeInt(IQ_CURRENT_LIMIT_EEPROM, PID.Iq_current_limit);
  writeFloat(KIID_EEPROM, PID.Ki_id);
  writeFloat(KPID_EEPROM, PID.Kp_id);

  writeInt(ID_CURRENT_LIMIT_EEPROM, PID.Id_current_limit);
  writeFloat(KP_EEPROM, PID.KP);
  writeFloat(KD_EEPROM, PID.KD);

  writeInt(CALIBRATED_EEPROM, controller.Calibrated);
  writeInt(PHASE_ORDER_EEPROM,controller.Phase_order);
  writeInt(WATCHDOG_TIME_EEPROM,controller.watchdog_time_ms);
  writeInt(WATCHDOG_ACTION_EEPROM,controller.watchdog_action);
  writeInt(HEARTBEAT_RATE_EEPROM,controller.Heartbeat_rate_ms);

  writeInt(I_AM_GRIPPER_EEPROM,controller.I_AM_GRIPPER);
  writeInt(RESET_INTEGRAL_EEPROM,PID.Reset_integral_accumulator);

  writeInt(TEMPERATURE_ERROR,controller.Max_temperature);
  writeInt(VOLTAGE_ERROR,controller.Max_Vbus);
  writeFloat(THETA_OFFSET, controller.theta_offset);
  writeInt(VOLTAGE_LIMIT,PID.Voltage_limit);

}

/// @brief  Reset to default config.  First write to EEPROM then read it back.
void Set_Default_config(){

  writeInt(SERIAL_NUMBER_EEPROM,1111);
  writeInt(HARDWARE_VERSION_EEPROM, 1);
  writeInt(BATCH_DATA_EEPROM, 24092024);

  writeInt(CAN_ID_EEPROM, 0);
  writeInt(SOFTWARE_VERSION_EEPROM, 101);
  writeInt(LED_ON_OFF_EEPROM, 1);
  writeInt(THERMISTOR_ON_OFF_EEPROM, 0);
  writeInt(POLE_PAIR, 7);
  writeInt(DIR_EEPROM, 0);
  writeInt(PHASE_ORDER_EEPROM, 2);
  writeFloat(RESISTANCE_EEPROM, 0);
  writeFloat(TOTAL_RESISTANCE_EEPROM, 0);
  writeFloat(INDUCTANCE_EEPROM, 0);

  writeFloat(KT_EEPROM, 0);
  writeFloat(KV_EEPROM, 0);
  writeFloat(FLUX_LINKAGE_EEPROM, 0);
  writeFloat(KPP_EEPROM, 11);
  writeFloat(KPV_EEPROM, 0.03);
  writeFloat(KIV_EEPROM, 0.0003);

  writeFloat(VELOCITY_LIMIT_EEPROM, 800000);
  writeFloat(KIIQ_EEPROM, 1.5);
  writeFloat(KPIQ_EEPROM, 3);

  writeInt(IQ_CURRENT_LIMIT_EEPROM, 1000);
  writeFloat(KIID_EEPROM, 1.5);
  writeFloat(KPID_EEPROM, 3);

  writeInt(ID_CURRENT_LIMIT_EEPROM, 0);
  writeFloat(KD_EEPROM, 0.0028);
  writeFloat(KP_EEPROM, 0.1400);

  writeInt(CALIBRATED_EEPROM, 0);
  writeInt(PHASE_ORDER_EEPROM,0);
  writeInt(WATCHDOG_TIME_EEPROM,0);
  writeInt(WATCHDOG_ACTION_EEPROM,0);
  writeInt(HEARTBEAT_RATE_EEPROM,0);

  writeInt(I_AM_GRIPPER_EEPROM,0);
  writeInt(RESET_INTEGRAL_EEPROM,0);

  writeInt(TEMPERATURE_ERROR,75);
  writeInt(VOLTAGE_ERROR,29000);
  writeFloat(THETA_OFFSET, 0);
  writeInt(VOLTAGE_LIMIT,0);

  read_config();

}



void Write_cogging_map()
{
}

void Read_cogging_map()
{
}
