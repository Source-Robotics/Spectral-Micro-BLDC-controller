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

/// @brief CRC16-CCITT update, one byte at a time.
static uint16_t crc16_update(uint16_t crc, uint8_t data)
{
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; i++)
  {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
  }
  return crc;
}

/// @brief Fold the int32 EEPROM values at the given addresses into a running CRC.
static uint16_t crc16_over_ints(uint16_t crc, const unsigned int *addrs, size_t count)
{
  for (size_t n = 0; n < count; n++)
  {
    int32_t v = readInt(addrs[n]);
    const uint8_t *p = (const uint8_t *)&v;
    for (size_t i = 0; i < sizeof(v); i++)
      crc = crc16_update(crc, p[i]);
  }
  return crc;
}

/// @brief Fold the float EEPROM values at the given addresses into a running CRC.
static uint16_t crc16_over_floats(uint16_t crc, const unsigned int *addrs, size_t count)
{
  for (size_t n = 0; n < count; n++)
  {
    float v = readFloat(addrs[n]);
    const uint8_t *p = (const uint8_t *)&v;
    for (size_t i = 0; i < sizeof(v); i++)
      crc = crc16_update(crc, p[i]);
  }
  return crc;
}

// --- Calibration block ---
// Motor-specific data that costs real time (and physical access to the hardware) to
// regenerate: the raw measurements from calibration, plus the current-loop gains that are
// DERIVED from those measurements (Kp_iq/Ki_iq/Kp_id/Ki_id are computed from Resistance/
// Inductance during calibration -- see Update_IT_callback_calib) so a "generic" fallback
// value for them is motor-wrong, not just untuned. Validated by its OWN CRC, independent of
// Settings, so a future firmware update that only adds a new SETTING can never touch this
// block's fingerprint or force a recalibration.
// CALIB_RESERVED_*/CALIB_FEATURE_FLAGS are currently unused (always 0) -- pre-reserved
// headroom for future calibration-adjacent features. See the long comment above their
// #defines in constants.h for why they're included in this hash from the start, and the
// important caveat about not growing this list again once boards are deployed with a CRC
// sealed over it.
static const unsigned int CALIB_INT_ADDRS[] = {
    POLE_PAIR, DIR_EEPROM, PHASE_ORDER_EEPROM, CALIBRATED_EEPROM, COMMUTATION_DIR_EEPROM,
    CALIB_RESERVED_I1_EEPROM, CALIB_RESERVED_I2_EEPROM, CALIB_FEATURE_FLAGS_EEPROM};
static const unsigned int CALIB_FLOAT_ADDRS[] = {
    RESISTANCE_EEPROM, TOTAL_RESISTANCE_EEPROM, INDUCTANCE_EEPROM, KT_EEPROM, KV_EEPROM,
    FLUX_LINKAGE_EEPROM, THETA_OFFSET, KPIQ_EEPROM, KIIQ_EEPROM, KPID_EEPROM, KIID_EEPROM,
    CALIB_RESERVED_F1_EEPROM, CALIB_RESERVED_F2_EEPROM, CALIB_RESERVED_F3_EEPROM, CALIB_RESERVED_F4_EEPROM};

static uint16_t Compute_calibration_crc_from_eeprom()
{
  uint16_t crc = 0xFFFF;
  crc = crc16_over_ints(crc, CALIB_INT_ADDRS, sizeof(CALIB_INT_ADDRS) / sizeof(CALIB_INT_ADDRS[0]));
  crc = crc16_over_floats(crc, CALIB_FLOAT_ADDRS, sizeof(CALIB_FLOAT_ADDRS) / sizeof(CALIB_FLOAT_ADDRS[0]));
  return crc;
}

// --- Settings block ---
// Tunables with safe, motor-independent defaults (position/velocity/PD gains, current
// limits, watchdog config, CAN ID, etc). Losing this block costs re-tuning, not
// re-touching hardware -- so it's lower-stakes, but still worth preserving across upgrades
// rather than resetting for no reason.
static const unsigned int SETTINGS_INT_ADDRS[] = {
    CAN_ID_EEPROM, SOFTWARE_VERSION_EEPROM, LED_ON_OFF_EEPROM, THERMISTOR_ON_OFF_EEPROM,
    IQ_CURRENT_LIMIT_EEPROM, ID_CURRENT_LIMIT_EEPROM, WATCHDOG_TIME_EEPROM, WATCHDOG_ACTION_EEPROM,
    HEARTBEAT_RATE_EEPROM, I_AM_GRIPPER_EEPROM, RESET_INTEGRAL_EEPROM, TEMPERATURE_ERROR,
    VOLTAGE_ERROR, VOLTAGE_LIMIT};
static const unsigned int SETTINGS_FLOAT_ADDRS[] = {
    KPP_EEPROM, KPV_EEPROM, KIV_EEPROM, VELOCITY_LIMIT_EEPROM, KP_EEPROM, KD_EEPROM};

static uint16_t Compute_settings_crc_from_eeprom()
{
  uint16_t crc = 0xFFFF;
  crc = crc16_over_ints(crc, SETTINGS_INT_ADDRS, sizeof(SETTINGS_INT_ADDRS) / sizeof(SETTINGS_INT_ADDRS[0]));
  crc = crc16_over_floats(crc, SETTINGS_FLOAT_ADDRS, sizeof(SETTINGS_FLOAT_ADDRS) / sizeof(SETTINGS_FLOAT_ADDRS[0]));
  return crc;
}

/// @brief Load only the calibration-block fields from EEPROM into the live controller/PID
/// structs. Split out from read_config() so Set_Default_calibration_block() can call it too,
/// refreshing RAM immediately after writing fresh defaults -- otherwise the board would keep
/// running on the old (invalid) in-memory values until the next reboot.
static void Load_calibration_from_eeprom()
{
  controller.pole_pairs = readInt(POLE_PAIR);
  controller.DIR_ = readInt(DIR_EEPROM);
  controller.Phase_order = readInt(PHASE_ORDER_EEPROM);
  controller.Calibrated = readInt(CALIBRATED_EEPROM);
  // Stored as 2 = reversed, anything else (incl. erased 0xFFFFFFFF on legacy boards) = forward (+1)
  controller.commutation_dir = (readInt(COMMUTATION_DIR_EEPROM) == 2) ? -1 : 1;
  controller.Resistance = readFloat(RESISTANCE_EEPROM);
  controller.Total_Resistance = readFloat(TOTAL_RESISTANCE_EEPROM);
  controller.Inductance = readFloat(INDUCTANCE_EEPROM);
  controller.Kt = readFloat(KT_EEPROM);
  controller.KV = readFloat(KV_EEPROM);
  controller.flux_linkage = readFloat(FLUX_LINKAGE_EEPROM);
  controller.theta_offset = readFloat(THETA_OFFSET);
  PID.Kp_iq = readFloat(KPIQ_EEPROM);
  PID.Ki_iq = readFloat(KIIQ_EEPROM);
  PID.Kp_id = readFloat(KPID_EEPROM);
  PID.Ki_id = readFloat(KIID_EEPROM);
}

/// @brief Load only the settings-block fields from EEPROM into the live controller/PID
/// structs. Split out from read_config() so Set_Default_settings_block() can call it too,
/// refreshing RAM immediately after writing fresh defaults.
static void Load_settings_from_eeprom()
{
  controller.CAN_ID = readInt(CAN_ID_EEPROM);
  if (!spectral_can_id_int_is_valid(controller.CAN_ID)) {
    controller.CAN_ID = 0;
  }
  // Report the firmware's own build version, not the (possibly stale) value in EEPROM.
  // This way #Info reflects the flashed firmware immediately, without needing a #Default.
  controller.SOFTWARE_VERSION = FIRMWARE_VERSION;
  controller.LED_ON_OFF = readInt(LED_ON_OFF_EEPROM);
  controller.Thermistor_on_off = readInt(THERMISTOR_ON_OFF_EEPROM);
  PID.Kp_p = readFloat(KPP_EEPROM);
  PID.Kp_v = readFloat(KPV_EEPROM);
  PID.Ki_v = readFloat(KIV_EEPROM);
  PID.Velocity_limit = readFloat(VELOCITY_LIMIT_EEPROM);
  PID.Iq_current_limit = readInt(IQ_CURRENT_LIMIT_EEPROM);
  PID.Id_current_limit = readInt(ID_CURRENT_LIMIT_EEPROM);
  PID.KP = readFloat(KP_EEPROM);
  PID.KD = readFloat(KD_EEPROM);
  controller.watchdog_time_ms = readInt(WATCHDOG_TIME_EEPROM);
  controller.watchdog_action = readInt(WATCHDOG_ACTION_EEPROM);
  controller.Heartbeat_rate_ms = readInt(HEARTBEAT_RATE_EEPROM);
  controller.I_AM_GRIPPER = readInt(I_AM_GRIPPER_EEPROM);
  PID.Reset_integral_accumulator = readInt(RESET_INTEGRAL_EEPROM);
  controller.Max_temperature = readInt(TEMPERATURE_ERROR);
  controller.Max_Vbus = readInt(VOLTAGE_ERROR);
  PID.Voltage_limit = readInt(VOLTAGE_LIMIT);
}

// Forward declarations: defined near Set_Default_config() below, but read_config() (which
// comes first in this file) needs to call them when a block's CRC doesn't validate.
static void Set_Default_calibration_block();
static void Set_Default_settings_block();

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

  Load_calibration_from_eeprom();
  Load_settings_from_eeprom();

  // Validate each block against its OWN stored CRC (see the Calibration/Settings block
  // comments above Compute_calibration_crc_from_eeprom()/Compute_settings_crc_from_eeprom()).
  // Keeping them independent means a future firmware update that only adds a new SETTING
  // can never invalidate the calibration block's fingerprint -- it structurally can't touch
  // it -- so it can never force a recalibration on hardware that may be hard to reach.
  //
  // One-time migration case (shared by both blocks): boards already in the field (this
  // firmware has shipped as V101-V105) have real, valid data but unwritten (erased = -1) CRC
  // slots, which look identical to "a fingerprint was never sealed" -- indistinguishable from
  // genuine corruption by the CRC alone. Disambiguate using Calibrated: Set_Default_config()
  // (and a completed calibration) always writes it as a real 0 or 1, while a truly virgin
  // EEPROM chip reads it back as -1 (erased). (Unlike the sibling stepper firmware, pole_pairs
  // itself can't be used here -- this firmware's own factory default is pole_pairs=7, which is
  // also a plausible real motor value, not a safe "never configured" sentinel.) Only in that
  // legacy-with-plausible-data case do we trust the existing values once and seal them with a
  // real CRC now, so any FUTURE genuine corruption is still caught normally.
  bool looks_like_real_legacy_data = (controller.Calibrated == 0 || controller.Calibrated == 1);

  int32_t stored_calib_crc = readInt(CALIBRATION_CRC_EEPROM);
  if (stored_calib_crc == -1 && looks_like_real_legacy_data)
  {
    writeInt(CALIBRATION_CRC_EEPROM, (int32_t)Compute_calibration_crc_from_eeprom());
  }
  else if (Compute_calibration_crc_from_eeprom() != (uint16_t)stored_calib_crc)
  {
    Set_Default_calibration_block();
  }

  int32_t stored_settings_crc = readInt(SETTINGS_CRC_EEPROM);
  if (stored_settings_crc == -1 && looks_like_real_legacy_data)
  {
    writeInt(SETTINGS_CRC_EEPROM, (int32_t)Compute_settings_crc_from_eeprom());
  }
  else if (Compute_settings_crc_from_eeprom() != (uint16_t)stored_settings_crc)
  {
    Set_Default_settings_block();
  }
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
  writeInt(COMMUTATION_DIR_EEPROM, (controller.commutation_dir < 0) ? 2 : 1);

  writeInt(CALIBRATION_CRC_EEPROM, (int32_t)Compute_calibration_crc_from_eeprom());
  writeInt(SETTINGS_CRC_EEPROM, (int32_t)Compute_settings_crc_from_eeprom());

}

/// @brief Reset just the calibration block to safe defaults and reseal its CRC. Used when
/// its CRC doesn't validate, or as part of a full Set_Default_config().
static void Set_Default_calibration_block()
{
  writeInt(POLE_PAIR, 7);
  writeInt(DIR_EEPROM, 0);
  writeInt(PHASE_ORDER_EEPROM, 0);
  writeInt(CALIBRATED_EEPROM, 0);
  writeInt(COMMUTATION_DIR_EEPROM, 1);
  writeFloat(RESISTANCE_EEPROM, 0);
  writeFloat(TOTAL_RESISTANCE_EEPROM, 0);
  writeFloat(INDUCTANCE_EEPROM, 0);
  writeFloat(KT_EEPROM, 0);
  writeFloat(KV_EEPROM, 0);
  writeFloat(FLUX_LINKAGE_EEPROM, 0);
  writeFloat(THETA_OFFSET, 0);
  writeFloat(KPIQ_EEPROM, 3);
  writeFloat(KIIQ_EEPROM, 1.5);
  writeFloat(KPID_EEPROM, 3);
  writeFloat(KIID_EEPROM, 1.5);

  // Reserved, not currently used by anything -- see the #defines in constants.h.
  writeFloat(CALIB_RESERVED_F1_EEPROM, 0);
  writeFloat(CALIB_RESERVED_F2_EEPROM, 0);
  writeFloat(CALIB_RESERVED_F3_EEPROM, 0);
  writeFloat(CALIB_RESERVED_F4_EEPROM, 0);
  writeInt(CALIB_RESERVED_I1_EEPROM, 0);
  writeInt(CALIB_RESERVED_I2_EEPROM, 0);
  writeInt(CALIB_FEATURE_FLAGS_EEPROM, 0);

  writeInt(CALIBRATION_CRC_EEPROM, (int32_t)Compute_calibration_crc_from_eeprom());

  // Refresh RAM immediately: this can be called mid-read_config() (from within its CRC
  // check), which already read the OLD/invalid values into controller/PID before this ran.
  // Without this, the board would keep running on those stale values until the next reboot.
  Load_calibration_from_eeprom();
}

/// @brief Reset just the settings block to safe defaults and reseal its CRC. Used when its
/// CRC doesn't validate, or as part of a full Set_Default_config().
static void Set_Default_settings_block()
{
  writeInt(CAN_ID_EEPROM, 0);
  writeInt(SOFTWARE_VERSION_EEPROM, FIRMWARE_VERSION);
  writeInt(LED_ON_OFF_EEPROM, 1);
  writeInt(THERMISTOR_ON_OFF_EEPROM, 0);
  writeInt(IQ_CURRENT_LIMIT_EEPROM, 1000);
  writeInt(ID_CURRENT_LIMIT_EEPROM, 0);
  writeInt(WATCHDOG_TIME_EEPROM, 0);
  writeInt(WATCHDOG_ACTION_EEPROM, 0);
  writeInt(HEARTBEAT_RATE_EEPROM, 0);
  writeInt(I_AM_GRIPPER_EEPROM, 0);
  writeInt(RESET_INTEGRAL_EEPROM, 0);
  writeInt(TEMPERATURE_ERROR, 75);
  writeInt(VOLTAGE_ERROR, 29000);
  writeInt(VOLTAGE_LIMIT, 0);
  writeFloat(KPP_EEPROM, 11);
  writeFloat(KPV_EEPROM, 0.03);
  writeFloat(KIV_EEPROM, 0.0003);
  writeFloat(VELOCITY_LIMIT_EEPROM, 800000);
  writeFloat(KP_EEPROM, 0.1400);
  writeFloat(KD_EEPROM, 0.0028);

  writeInt(SETTINGS_CRC_EEPROM, (int32_t)Compute_settings_crc_from_eeprom());

  // See the matching comment in Set_Default_calibration_block(): refresh RAM immediately.
  Load_settings_from_eeprom();
}

/// @brief Reset ALL config (calibration + settings) to defaults, e.g. via the #Default
/// command. First write to EEPROM then read it back.
void Set_Default_config(){

  writeInt(SERIAL_NUMBER_EEPROM,1111);
  writeInt(HARDWARE_VERSION_EEPROM, 1);
  writeInt(BATCH_DATA_EEPROM, 24092024);

  Set_Default_calibration_block();
  Set_Default_settings_block();

  read_config();

}



void Write_cogging_map()
{
}

void Read_cogging_map()
{
}
