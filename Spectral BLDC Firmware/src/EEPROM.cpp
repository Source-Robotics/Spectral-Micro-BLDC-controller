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
#include "watchdog_action.h"
#include "cogging.h"
#include "encal.h"

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

// --- Settings block 2 (see constants.h: block 1 is frozen; new settings go here) ---
static const unsigned int SETTINGS2_INT_ADDRS[] = {
    TOOL_ID_EEPROM, WD_FALLBACK_TIME_EEPROM, WD_FALLBACK_CURRENT_EEPROM, WD_FALLBACK_SPEED_EEPROM,
    WD_AFTER_ACTION_EEPROM, BOARD_PRODUCT_EEPROM, CALIBRATION_ID_EEPROM,
    BRAKE_COAST_EEPROM, SETTINGS2_RESERVED_I4_EEPROM, SETTINGS2_FEATURE_FLAGS_EEPROM};
static const unsigned int SETTINGS2_FLOAT_ADDRS[] = {
    SETTINGS2_RESERVED_F1_EEPROM, SETTINGS2_RESERVED_F2_EEPROM, SETTINGS2_RESERVED_F3_EEPROM,
    SETTINGS2_RESERVED_F4_EEPROM};

static uint16_t Compute_settings2_crc_from_eeprom()
{
  uint16_t crc = 0xFFFF;
  crc = crc16_over_ints(crc, SETTINGS2_INT_ADDRS, sizeof(SETTINGS2_INT_ADDRS) / sizeof(SETTINGS2_INT_ADDRS[0]));
  crc = crc16_over_floats(crc, SETTINGS2_FLOAT_ADDRS, sizeof(SETTINGS2_FLOAT_ADDRS) / sizeof(SETTINGS2_FLOAT_ADDRS[0]));
  return crc;
}

/// @brief Block 2 defaults in RAM only: used when the EEPROM has no second bank, and as the values
/// Set_Default_settings2_block() writes.
static void Settings2_defaults_to_ram()
{
  controller.calibration_id = 0;
  controller.tool_id = 0;
  controller.wd_fallback_time_ms = 2000;
  controller.wd_fallback_current = 0;
  controller.wd_fallback_speed = 1000;
  controller.wd_after_action = WD_ACTION_LEGACY;
  controller.board_product = 0;
  controller.brake_coast = BRAKE_COAST_DEFAULT;
}

/// @brief Clamp the block 2 values to what the firmware accepts (also used for values set over CAN/UART).
void Settings2_sanitize()
{
  if (controller.tool_id < 0 || controller.tool_id > 255)
    controller.tool_id = 0;
  if (controller.wd_fallback_time_ms < 0)
    controller.wd_fallback_time_ms = 0;
  if (controller.wd_fallback_time_ms > WD_FALLBACK_TIME_MAX_MS)
    controller.wd_fallback_time_ms = WD_FALLBACK_TIME_MAX_MS;
  if (controller.wd_fallback_current < -32768 || controller.wd_fallback_current > 32767)
    controller.wd_fallback_current = 0;
  if (controller.wd_fallback_speed < 0 || controller.wd_fallback_speed > 65535)
    controller.wd_fallback_speed = 1000;
  if (controller.wd_after_action != WD_ACTION_BRAKE && controller.wd_after_action != WD_ACTION_COAST)
    controller.wd_after_action = WD_ACTION_LEGACY;
  if (controller.board_product != 1 && controller.board_product != 2)
    controller.board_product = 0;
}

static void Load_settings2_from_eeprom()
{
  controller.tool_id = readInt(TOOL_ID_EEPROM);
  controller.calibration_id = (uint32_t)readInt(CALIBRATION_ID_EEPROM);
  controller.wd_fallback_time_ms = readInt(WD_FALLBACK_TIME_EEPROM);
  controller.wd_fallback_current = readInt(WD_FALLBACK_CURRENT_EEPROM);
  controller.wd_fallback_speed = readInt(WD_FALLBACK_SPEED_EEPROM);
  controller.wd_after_action = readInt(WD_AFTER_ACTION_EEPROM);
  controller.board_product = readInt(BOARD_PRODUCT_EEPROM);
  {
    // 1 coast, 2 brake; anything else (0 on a block from before it was saved): the firmware's default
    const int saved = readInt(BRAKE_COAST_EEPROM);
    controller.brake_coast = saved == 2 ? 1 : saved == 1 ? 0 : BRAKE_COAST_DEFAULT;
  }
  Settings2_sanitize();
}

/// @brief Write block 2's defaults and seal its CRC. Only when the second bank exists.
static void Set_Default_settings2_block()
{
  Settings2_defaults_to_ram();
  if (!controller.eeprom_block2_ok)
    return;
  writeInt(TOOL_ID_EEPROM, controller.tool_id);
  writeInt(WD_FALLBACK_TIME_EEPROM, controller.wd_fallback_time_ms);
  writeInt(WD_FALLBACK_CURRENT_EEPROM, controller.wd_fallback_current);
  writeInt(WD_FALLBACK_SPEED_EEPROM, controller.wd_fallback_speed);
  writeInt(WD_AFTER_ACTION_EEPROM, controller.wd_after_action);
  writeInt(BOARD_PRODUCT_EEPROM, 0);
  writeInt(CALIBRATION_ID_EEPROM, 0);
  writeInt(BRAKE_COAST_EEPROM, 0); // not set: the default applies
  writeInt(SETTINGS2_RESERVED_I4_EEPROM, 0);
  writeFloat(SETTINGS2_RESERVED_F1_EEPROM, 0);
  writeFloat(SETTINGS2_RESERVED_F2_EEPROM, 0);
  writeFloat(SETTINGS2_RESERVED_F3_EEPROM, 0);
  writeFloat(SETTINGS2_RESERVED_F4_EEPROM, 0);
  writeInt(SETTINGS2_FEATURE_FLAGS_EEPROM, 0);
  writeInt(SETTINGS2_CRC_EEPROM, (int32_t)Compute_settings2_crc_from_eeprom());
}

// --- Product lock (2026-11) ---
// STEPFOC and Spectral share the bootloader and the file name, and each firmware drives the other board's pins
// wrongly (the Spectral image on a STEPFOC drove a coil at ~1.7 A in every mode). A board remembers which product
// calibrated it (BOARD_PRODUCT_EEPROM, set when a calibrated board is saved or at the first start-up of a
// calibrated board); a firmware that finds the other product's stamp, or no stamp but a calibration this product
// cannot produce, touches nothing: see setup().

/// The pole pairs a calibration by this product can produce.
static bool Pole_pairs_fit_product(int32_t pp)
{
  return pp >= 1 && pp < 50; // BLDC motors; a STEPFOC calibration is always 50, 75, 100 or 125
}

/// Stamp the board in RAM when this firmware calibrated it (plausibly) and it has no stamp yet.
static void Product_stamp_if_calibrated()
{
  if (controller.board_product == 0 && controller.Calibrated == 1 && Pole_pairs_fit_product(controller.pole_pairs))
    controller.board_product = PRODUCT_ID;
}

/// At start-up, BEFORE read_config(): is this the other product's board? Only reads, never writes, so a firmware
/// on the wrong board can never "repair" (overwrite) that board's configuration.
bool Product_board_is_other()
{
  int32_t stamp = controller.eeprom_block2_ok ? readInt(BOARD_PRODUCT_EEPROM) : 0;
  if (stamp == 1 || stamp == 2)
  {
    controller.board_product = stamp;
    return stamp != PRODUCT_ID;
  }
  int32_t calibrated_raw = readInt(CALIBRATED_EEPROM);
  int32_t pole_pairs = readInt(POLE_PAIR);
  return calibrated_raw == 1 && !Pole_pairs_fit_product(pole_pairs);
}

/// Wrong-product mode: only the CAN ID, so the board stays addressable (status, identity, reset into bootloader).
void Load_can_id_only()
{
  controller.CAN_ID = readInt(CAN_ID_EEPROM);
  if (!spectral_can_id_int_is_valid(controller.CAN_ID))
    controller.CAN_ID = 0;
}

/// Set or clear the stamp in EEPROM and reseal block 2 (start-up stamping, #Product claim / clear).
void Product_set_stamp(int product)
{
  controller.board_product = product;
  if (!controller.eeprom_block2_ok)
    return;
  writeInt(BOARD_PRODUCT_EEPROM, product);
  writeInt(SETTINGS2_CRC_EEPROM, (int32_t)Compute_settings2_crc_from_eeprom());
}

/// After read_config() on the right board: a calibrated board without a stamp gets one, once.
void Product_stamp_at_boot()
{
  int before = controller.board_product;
  Product_stamp_if_calibrated();
  if (controller.board_product != before)
    Product_set_stamp(controller.board_product);
}

/// @brief Write the block 2 values in RAM and reseal its CRC (part of Write_config).
static void Write_settings2_block()
{
  if (!controller.eeprom_block2_ok)
    return;
  Product_stamp_if_calibrated();
  Settings2_sanitize();
  writeInt(BOARD_PRODUCT_EEPROM, controller.board_product);
  writeInt(TOOL_ID_EEPROM, controller.tool_id);
  writeInt(CALIBRATION_ID_EEPROM, (int32_t)controller.calibration_id);
  writeInt(WD_FALLBACK_TIME_EEPROM, controller.wd_fallback_time_ms);
  writeInt(WD_FALLBACK_CURRENT_EEPROM, controller.wd_fallback_current);
  writeInt(WD_FALLBACK_SPEED_EEPROM, controller.wd_fallback_speed);
  writeInt(WD_AFTER_ACTION_EEPROM, controller.wd_after_action);
  writeInt(BRAKE_COAST_EEPROM, controller.brake_coast ? 2 : 1); // parameter 18: it was lost at every power-up
  writeInt(SETTINGS2_CRC_EEPROM, (int32_t)Compute_settings2_crc_from_eeprom());
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

  // Settings block 2 lives at 256+, the second 256-byte bank of a 24C16, which answers at I2C address
  // 0x51 (the library puts address bits 8-10 into the device address). A smaller chip would not answer
  // there; then block 2 is never read or written and its settings keep their defaults in RAM.
  Wire.beginTransmission(DEVICEADDRESS | 0x01);
  controller.eeprom_block2_ok = (Wire.endTransmission() == 0);
  // Anti-cogging block 3 reaches into the fourth bank (0x53).
  Wire.beginTransmission(DEVICEADDRESS | 0x03);
  controller.eeprom_block3_ok = (Wire.endTransmission() == 0);
  // The encoder block (encal.h) reaches into the fifth bank (0x54).
  Wire.beginTransmission(DEVICEADDRESS | 0x04);
  controller.eeprom_block4_ok = (Wire.endTransmission() == 0);
}

/// A block's CRC check, believed "wrong" only after three readings in a row. One misread on the I2C bus used to be
/// enough for the block's defaults to be written over a stored calibration. After a first mismatch the block is
/// also loaded into RAM again: its values may come from the same bad reading.
static bool Block_crc_ok(uint16_t (*compute)(), unsigned int crc_address, void (*reload)())
{
  for (int attempt = 0; attempt < 3; attempt++)
  {
    if (attempt > 0)
    {
      delay(2);
      reload();
    }
    if (compute() == (uint16_t)readInt(crc_address))
      return true;
  }
  return false;
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
  //
  // 2026-11: the test used to read controller.Calibrated, which is a bool: readInt()'s -1 from a
  // virgin EEPROM narrowed to true, so the test was always true and a blank chip's -1s and NaNs were
  // sealed as valid settings. The RAW int is what tells "saved at least once" (0 or 1) from "never
  // written" (-1); every board that was ever saved is still trusted, exactly as before.
  int32_t calibrated_raw = readInt(CALIBRATED_EEPROM);
  bool looks_like_real_legacy_data = (calibrated_raw == 0 || calibrated_raw == 1);

  int32_t stored_calib_crc = readInt(CALIBRATION_CRC_EEPROM);
  if (stored_calib_crc == -1 && looks_like_real_legacy_data)
  {
    writeInt(CALIBRATION_CRC_EEPROM, (int32_t)Compute_calibration_crc_from_eeprom());
  }
  else if (!Block_crc_ok(Compute_calibration_crc_from_eeprom, CALIBRATION_CRC_EEPROM, Load_calibration_from_eeprom))
  {
    Set_Default_calibration_block();
  }

  int32_t stored_settings_crc = readInt(SETTINGS_CRC_EEPROM);
  if (stored_settings_crc == -1 && looks_like_real_legacy_data)
  {
    writeInt(SETTINGS_CRC_EEPROM, (int32_t)Compute_settings_crc_from_eeprom());
  }
  else if (!Block_crc_ok(Compute_settings_crc_from_eeprom, SETTINGS_CRC_EEPROM, Load_settings_from_eeprom))
  {
    Set_Default_settings_block();
  }

  // Settings block 2: no legacy case, its fields never existed before, so an unsealed or mismatched
  // block simply gets its defaults. Nothing older is touched.
  if (controller.eeprom_block2_ok)
  {
    Load_settings2_from_eeprom();
    if (!Block_crc_ok(Compute_settings2_crc_from_eeprom, SETTINGS2_CRC_EEPROM, Load_settings2_from_eeprom))
    {
      Set_Default_settings2_block();
    }
  }
  else
  {
    Settings2_defaults_to_ram();
  }

  Read_cogging_map();
  Read_encoder_map();
}

/// @brief Clean the config that is saved in the EEPROM
void Clean_config()
{
}

/// @brief Write latest data to the EEPROM
/// @details
void Write_config()
{
  if (controller.wrong_product) // never write the other product's configuration
    return;

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
  Write_settings2_block();
  Write_cogging_map();
  Write_encoder_map();
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
  // The CAN ID is the one field here whose loss is not "costs re-tuning": a board that drops to node 0
  // collides with every other board that did the same, which needs physical access to untangle. Keep a
  // valid stored ID, in both callers: the automatic reset after a bad CRC and #Default. Fall back to 0
  // only when nothing usable is stored (a blank EEPROM reads -1).
  int32_t existing_id = readInt(CAN_ID_EEPROM);
  if (!spectral_can_id_int_is_valid(existing_id))
  {
    existing_id = 0;
  }
  writeInt(CAN_ID_EEPROM, existing_id);
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
  if (controller.wrong_product) // never write the other product's configuration
    return;

  writeInt(SERIAL_NUMBER_EEPROM,1111);
  writeInt(HARDWARE_VERSION_EEPROM, 1);
  writeInt(BATCH_DATA_EEPROM, 24092024);

  Set_Default_calibration_block();
  Set_Default_settings_block();
  Set_Default_settings2_block();
  Cogging_defaults();
  Write_cogging_map();
  Encoder_defaults(); // a reset drive may sit on another motor
  Write_encoder_map();

  read_config();

}



/// CRC-16 of the anti-cogging block as it is in RAM
static uint16_t Cogging_crc(const CoggingBlock *block)
{
  uint16_t crc = 0xFFFF;
  const uint8_t *p = (const uint8_t *)block;
  for (size_t i = 0; i < sizeof(CoggingBlock); i++)
    crc = crc16_update(crc, p[i]);
  return crc;
}

/// The CRC of block 3 as it was last saved or loaded.
static uint16_t cogging_saved_crc = 0;

/// @brief True while the anti-cogging in RAM is what block 3 holds: saved or loaded, and not changed since. A new
/// identification, or a changed mode or scale, makes it false until the next save (#Info used to keep saying "saved").
bool Cogging_is_saved()
{
  return controller.cogging_saved && Cogging_crc(&cogging) == cogging_saved_crc;
}

/// @brief Save anti-cogging block 3: only the 16-byte pages that changed are written, then the CRC.
void Write_cogging_map()
{
  if (!controller.eeprom_block3_ok || controller.wrong_product)
    return;
  Cogging_sanitize();
  const uint8_t *p = (const uint8_t *)&cogging;
  unsigned int done = 0;
  while (done < sizeof(CoggingBlock))
  {
    const unsigned int addr = COGGING_BLOCK_EEPROM + done;
    unsigned int n = 16 - (addr % 16); // never across a page (or bank) boundary
    if (n > sizeof(CoggingBlock) - done)
      n = sizeof(CoggingBlock) - done;
    uint8_t old[16];
    eeprom.readBlock(addr, old, n);
    if (memcmp(old, p + done, n) != 0)
      eeprom.writeBlock(addr, p + done, n);
    done += n;
  }
  const int32_t crc = (int32_t)Cogging_crc(&cogging);
  if (readInt(COGGING_CRC_EEPROM) != crc) // a save with nothing changed writes nothing
    writeInt(COGGING_CRC_EEPROM, crc);
  cogging_saved_crc = (uint16_t)crc;
  controller.cogging_saved = 1;
}

/// @brief Load anti-cogging block 3; off (defaults) if the bank is missing or the CRC does not match.
void Read_cogging_map()
{
  controller.cogging_saved = 0;
  if (!controller.eeprom_block3_ok)
  {
    Cogging_defaults();
    return;
  }
  CoggingBlock block;
  uint8_t *p = (uint8_t *)&block;
  for (int attempt = 0; attempt < 3; attempt++) // one misread must not switch the anti-cogging off until the next start
  {
    unsigned int done = 0;
    while (done < sizeof(CoggingBlock))
    {
      const unsigned int addr = COGGING_BLOCK_EEPROM + done;
      unsigned int n = 16 - (addr % 16);
      if (n > sizeof(CoggingBlock) - done)
        n = sizeof(CoggingBlock) - done;
      eeprom.readBlock(addr, p + done, n);
      done += n;
    }
    const uint32_t stored = (uint32_t)readInt(COGGING_CRC_EEPROM);
    if ((stored >> 16) == 0 && (uint16_t)stored == Cogging_crc(&block) && block.version == COGGING_BLOCK_VERSION)
    {
      const uint32_t irq = __get_PRIMASK();
      __disable_irq(); // the control loop may be running (Set_Default_config)
      memcpy(&cogging, &block, sizeof(CoggingBlock));
      Cogging_sanitize();
      __set_PRIMASK(irq);
      cogging_saved_crc = Cogging_crc(&cogging);
      controller.cogging_saved = 1;
      return;
    }
    delay(2);
  }
  Cogging_defaults();
}

/// CRC-16 of the encoder block as it is in RAM
static uint16_t Encoder_crc(const EncoderBlock *block)
{
  uint16_t crc = 0xFFFF;
  const uint8_t *p = (const uint8_t *)block;
  for (size_t i = 0; i < sizeof(EncoderBlock); i++)
    crc = crc16_update(crc, p[i]);
  return crc;
}

/// @brief Save the encoder block (encal.h) after anti-cogging block 3: only the 16-byte pages that changed, then the CRC.
void Write_encoder_map()
{
  if (!controller.eeprom_block3_ok || !controller.eeprom_block4_ok || controller.wrong_product)
    return;
  Encoder_sanitize();
  const uint8_t *p = (const uint8_t *)&encoder;
  unsigned int done = 0;
  while (done < sizeof(EncoderBlock))
  {
    const unsigned int addr = ENCODER_BLOCK_EEPROM + done;
    unsigned int n = 16 - (addr % 16); // never across a page (or bank) boundary
    if (n > sizeof(EncoderBlock) - done)
      n = sizeof(EncoderBlock) - done;
    uint8_t old[16];
    eeprom.readBlock(addr, old, n);
    if (memcmp(old, p + done, n) != 0)
      eeprom.writeBlock(addr, p + done, n);
    done += n;
  }
  const int32_t crc = (int32_t)Encoder_crc(&encoder);
  if (readInt(ENCODER_CRC_EEPROM) != crc)
    writeInt(ENCODER_CRC_EEPROM, crc);
}

/// @brief Load the encoder block; no table and not in use if the banks are missing or the CRC does not match.
void Read_encoder_map()
{
  if (!controller.eeprom_block3_ok || !controller.eeprom_block4_ok)
  {
    Encoder_defaults();
    return;
  }
  EncoderBlock block;
  uint8_t *p = (uint8_t *)&block;
  for (int attempt = 0; attempt < 3; attempt++) // one misread must not switch the encoder table off until the next start
  {
    unsigned int done = 0;
    while (done < sizeof(EncoderBlock))
    {
      const unsigned int addr = ENCODER_BLOCK_EEPROM + done;
      unsigned int n = 16 - (addr % 16);
      if (n > sizeof(EncoderBlock) - done)
        n = sizeof(EncoderBlock) - done;
      eeprom.readBlock(addr, p + done, n);
      done += n;
    }
    const uint32_t stored = (uint32_t)readInt(ENCODER_CRC_EEPROM);
    if ((stored >> 16) == 0 && (uint16_t)stored == Encoder_crc(&block) && block.version == ENCODER_BLOCK_VERSION)
    {
      const uint32_t irq = __get_PRIMASK();
      __disable_irq(); // the control loop may be running (Set_Default_config)
      memcpy(&encoder, &block, sizeof(EncoderBlock));
      Encoder_sanitize();
      __set_PRIMASK(irq);
      return;
    }
    delay(2);
  }
  Encoder_defaults();
}
