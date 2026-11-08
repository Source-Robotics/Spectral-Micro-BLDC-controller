/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    utils.h
  * @brief   This file contains all the function prototypes for math utilities and functions 
  * for utils.cpp file
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
#ifndef UTILS_
#define UTILS_

#include <Arduino.h>
#include <stdio.h>
#include "iodefs.h"
#include "hw_init.h"
#include "qfplib-m3.h"


void limit_norm(volatile float *x, volatile float *y, volatile float limit);
float fmaxf(float x, float y);
float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
void Ticker_init(TIM_TypeDef *Instance, int frequency,  void (*int_callback)());
void Ticker_detach(TIM_TypeDef *Instance);
float movingAverage_f(float value);
int movingAverage(int value);
float map_float(float x, float in_min, float in_max, float out_min, float out_max);
float EMA_f();
int EMA();
float fast_modf2(float x);
int64_t maxInt3(int64_t x, int64_t y, int64_t z); 
void intTo2Bytes(int32_t value, byte *bytes);
void intTo4Bytes(int32_t value, byte *bytes);
void intTo3Bytes(int32_t value, byte *bytes);
unsigned char bitsToByte(const bool *bits);
void byteToBits(byte b, bool *bits);
void byteToBitsBigEndian(byte b, bool* bits);
int three_bytes_to_int(uint8_t *bytes);
int two_bytes_to_int(uint8_t *bytes);
int fourBytesToInt(uint8_t *bytes);
float fourBytesToFloat(uint8_t *bytes);
byte mapAndConstrain(int value,int upper_limit, int lower_limit);
bool isAroundValue(int value, int setpoint, int Tolerance);
float parse_float(const char *s);                  // atof without strtod
int parse_ints(const char *s, int *out, int n);    // "a,b,c" -> how many read
const char *next_word(const char *p, char *out, int size); // the next word, sscanf "%s" without scanf
float wrap_2pi(float x);                           // into [0, 2 pi), fmod without double maths

#endif
