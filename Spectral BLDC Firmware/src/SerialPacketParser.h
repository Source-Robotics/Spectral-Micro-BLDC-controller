/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    SerialPacketParser.h
  * @brief   This file contains all the function prototypes for the SerialPacketParser.cpp file
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
#ifndef SerialPacketParser_h
#define SerialPacketParser_h

#include <Arduino.h>

class SerialPacketParser {
public:
  SerialPacketParser();
  bool parse(char c, char *command, char *argument);
  bool handleIq(char *argument);

  void resetCommandAndArgument(char *command, char *argument);

  // Set true when a complete line was received but rejected (too long, malformed, or not
  // in the whitelist below); left false while a packet is still being buffered. Caller
  // should check this after parse() returns false and clear it once handled.
  bool lastCommandUnknown = false;

private:
  char buffer[64];
  int bufferIndex;
  bool packetComplete;
};

#endif