/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    utils.cpp
  * @brief   This file provides code where we define math functions and utilities
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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "utils.h"
#include <Arduino.h>
#include <stdio.h>
#include "iodefs.h"
#include "hw_init.h"
#include "qfplib-m3.h"

/*
amt: The value to be constrained.
low: The lower bound of the acceptable range.
high: The upper bound of the acceptable range.

The macro is used to limit the value (amt) to be within the specified range defined by low and high.
 If amt is less than low, it is replaced by low. If amt is greater than high, it is replaced by high. 
 If amt is within the specified range, it remains unchanged.
*/
#define constrain_value(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

/// @brief (Float) Returns maximum of x, y
/// @param x 
/// @param y 
/// @return maximum of x, y
float fmaxf(float x, float y){
    return (((x)>(y))?(x):(y));
    }

/// @brief (Float) Returns minimum of x, y
/// @param x 
/// @param y 
/// @return minimum of x, y
float fminf(float x, float y){
    return (((x)<(y))?(x):(y));
    }

/// @brief (Float) Returns maximum of x, y, z
/// @param x 
/// @param y 
/// @param z 
/// @return maximum of x, y, z
float fmaxf3(float x, float y, float z){
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
    }


/// @brief Find max of 3 64 bit integers
/// @param x 
/// @param y 
/// @param z 
/// @return Returns max of the 3
int64_t maxInt3(int64_t x, int64_t y, int64_t z) {
  return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}


/// @brief (Float) Returns minimum of x, y, z
/// @param x 
/// @param y 
/// @param z 
/// @return Returns minimum of x, y, z
float fminf3(float x, float y, float z){
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
    }

/// @brief Scale lenght of vector (x, y) to be <= limit
/// @param x 
/// @param y 
/// @param limit 
void limit_norm(volatile float *x, volatile float *y, volatile float limit){
    float norm = qfp_fsqrt(*x * *x + *y * *y);
    if(norm > limit){
        *x = *x *  qfp_fdiv(limit,norm);
        *y = *y * qfp_fdiv(limit,norm);
        }
    }

/** 
 * Inits Ticker
 * Inputs are Timer instance, tick frequency and callback function
 * @param[in] Instance
 * @param[in] frequency
 * @param[in] void (*int_callback)()
*/
void Ticker_init(TIM_TypeDef *Instance, int frequency, void (*int_callback)()){

  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setOverflow(frequency, HERTZ_FORMAT); 
  MyTim->attachInterrupt((*int_callback));
  MyTim->resume();

}

/// @brief Detach ticker 
/// @param Instance 
void Ticker_detach(TIM_TypeDef *Instance){

  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->detachInterrupt();
}

/// @brief Map x that is in range of in_min to in_max to the out_min, out_max
/// @param x 
/// @param in_min 
/// @param in_max 
/// @param out_min 
/// @param out_max 
/// @return 
float map_float(float x, float in_min, float in_max, float out_min, float out_max){
  
 //return qfp_fdiv(qfp_fmul((x - in_min), (0.0375)), (4048 - 692));
 return qfp_fadd(qfp_fdiv(qfp_fmul(qfp_fsub(x,in_min),qfp_fsub(out_max,out_min)),qfp_fsub(in_max,in_min)),out_min);


}




/// @brief Simple moving average function for float values.
/// @param value variable we perform moving average on (Float)
/// @return output of the filter
float movingAverage_f(float value) {
  const byte nvalues = 20;             // Moving average window size

  static byte current = 0;            // Index for current value
  static byte cvalues = 0;            // Count of values read (<= nvalues)
  static float sum = 0;               // Rolling sum
  static float values[nvalues];

  //sum += value;
  sum  = qfp_fadd(sum, value);
  // If the window is full, adjust the sum by deleting the oldest value
  if (cvalues == nvalues)
    //sum -= values[current];
    sum = qfp_fsub(sum, values[current]);
  values[current] = value;          // Replace the oldest with the latest

  if (++current >= nvalues)
    current = 0;

  if (cvalues < nvalues)
    cvalues += 1;

  return qfp_fdiv(sum, cvalues);
}

float fast_modf2(float x){
    x *= 0.5f;
    return 2.0 * ( x - std::floor(x));
}


/// @brief Simple moving average function for int values.
/// @param value variable we perform moving average on (Int)
/// @return output of the filter
int movingAverage(int value){

  const byte nvalues = 20;             // Moving average window size 4

  static byte current = 0;            // Index for current value
  static byte cvalues = 0;            // Count of values read (<= nvalues)
  static int sum = 0;               // Rolling sum
  static int values[nvalues];

  //sum += value;
  sum  = sum + value;
  // If the window is full, adjust the sum by deleting the oldest value
  if (cvalues == nvalues)
    //sum -= values[current];
    sum = sum - values[current];
  values[current] = value;          // Replace the oldest with the latest

  if (++current >= nvalues)
    current = 0;

  if (cvalues < nvalues)
    cvalues += 1;

  return sum /  cvalues;
}


/// @brief convert signed int to 2 bytes
/// @param value
/// @param bytes 2 bytes
void intTo2Bytes(int32_t value, byte *bytes)
{
    bytes[0] = (value >> 8) & 0xFF; // extract upper byte
    bytes[1] = value & 0xFF;        // extract lower byte
}

/// @brief convert signed int to 4 bytes
/// @param value
/// @param bytes 4 bytes
void intTo4Bytes(int32_t value, byte *bytes)
{
    bytes[0] = (value >> 24) & 0xFF; // extract the first (most significant) byte
    bytes[1] = (value >> 16) & 0xFF; // extract the second byte
    bytes[2] = (value >> 8) & 0xFF;  // extract the third byte
    bytes[3] = value & 0xFF;         // extract the fourth (least significant) byte
}

/// @brief convert signed int to 3 bytes
/// @param value
/// @param bytes 3 bytes
void intTo3Bytes(int32_t value, byte *bytes)
{

    bytes[0] = (value >> 16) & 0xFF; 
    bytes[1] = (value >> 8) & 0xFF;  
    bytes[2] = value & 0xFF;        
}

/// @brief  convert array of (8) bits to byte
/// @param bits
/// @return bits fused into a byte
unsigned char bitsToByte(const bool *bits)
{
    unsigned char byte = 0;
    for (int i = 0; i < 8; ++i)
    {
        if (bits[i])
        {
            byte |= (1 << (7 - i));
        }
    }
    return byte;
}

/// @brief  Convert byte to array of bits
/// @param b byte we want to convert
/// @param bits bits we will get
void byteToBits(byte b, bool *bits)
{
    for (int i = 0; i < 8; i++)
    {
        bits[i] = (b >> i) & 0x01;
    }
}


/// @brief  Convert byte to array of bits
/// @param b byte we want to convert
/// @param bits bits we will get
void byteToBitsBigEndian(byte b, bool* bits) {
  for (int i = 7; i >= 0; i--) {
    bits[i] = (b >> (7 - i)) & 0x01;
  }
}


// uint8_t bytes[] = {temp[j][0], temp[j][1], temp[j][2]};
/// @brief convert 3 bytes into a signed int, used for robot position and speed variables
/// @param bytes
/// @return int value of fused bytes
int three_bytes_to_int(uint8_t *bytes)
{
  int value = ((int)bytes[0] << 16) | ((int)bytes[1] << 8) | (int)bytes[2];
  if (value & 0x00800000) // (value & 0x00800000)
  { // sign extend if needed
    value |= 0xFF000000;
  }
  return value;
}

/// @brief convert 2 bytes into a signed int
/// @param bytes
/// @return signed int
int two_bytes_to_int(uint8_t *bytes)
{
  int value = ((int)bytes[0] << 8) | (int)bytes[1];
  if (value & 0x00008000)
  { // sign extend if needed
    value |= 0xFFFF0000;
  }
  return value;
}

/// @brief convert 4 bytes into a signed int
/// @param bytes
/// @return signed int
int fourBytesToInt(uint8_t *bytes) {
  uint32_t value = ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) |
                    ((uint32_t)bytes[2] << 8) | (uint32_t)bytes[3];

  return (int32_t)value; // two's complement reinterpretation, no shift-by-32 needed
}

/// @brief Convert array of 4 bytes to float
/// @param bytes 
/// @return float value
float fourBytesToFloat(uint8_t *bytes) {
    union {
        float floatValue;
        uint32_t intValue;
    } data;

    data.intValue = (uint32_t)bytes[0] << 24 |
                    (uint32_t)bytes[1] << 16 |
                    (uint32_t)bytes[2] << 8 |
                    (uint32_t)bytes[3];

    return data.floatValue;
}

/// @brief map value from the range of upper and lower limit
// to the range of 0 - 255. If value is bigger than lower or upper range
// Constrain it to 0 or 255
/// @param value 
/// @param upper_limit 
/// @param lower_limit 
/// @return constrained value
byte mapAndConstrain(int value,int upper_limit, int lower_limit) {
  // Map the value from the range ex. [-1000, 1500] to [0, 255]
  int mappedValue = map(value, lower_limit, upper_limit, 0, 255);
  // Constrain the mapped value to the range [0, 255]
  byte constrainedValue = constrain(mappedValue, 0, 255);
  return constrainedValue;
}

  /// @brief Check if the value is around the setpoint. tolerance is the
  /// Amount we can be off.
  /// @param value 
  /// @param setpoint 
  /// @param Tolerance 
  /// @return 
  bool isAroundValue(int value, int setpoint, int Tolerance)
  {
    return abs(value - setpoint) <= Tolerance;
  }

 