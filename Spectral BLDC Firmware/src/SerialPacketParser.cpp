/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    SerialPacketParser.cpp
 * @brief   This file provides code for serial parser utility
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

#include "SerialPacketParser.h"

SerialPacketParser::SerialPacketParser()
{
  bufferIndex = 0;
  packetComplete = false;
}

bool SerialPacketParser::parse(char c, char *command, char *argument)
{

  if (c == '#')
  {
    bufferIndex = 0;
    packetComplete = false;
    lastCommandUnknown = false;
  }

  if (bufferIndex < sizeof(buffer) - 1)
  {
    buffer[bufferIndex++] = c;
    buffer[bufferIndex] = '\0'; // Null-terminate the buffer

    if (c == '\n')
    {
      packetComplete = true;
    }
  }

  if (packetComplete)
  {
    // Reset the buffer and flag after successfully processing a command
    bufferIndex = 0;
    packetComplete = false;

    // Extract the command and argument
    if (sscanf(buffer, "#%19s %19s", command, argument) == 2)
    {
      // Dispatch based on the parsed command
      if (strlen(command) + strlen(argument) <= 20)
      {

        // Dispatch based on the parsed command
        if (strcmp(command, "Iq") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Id") == 0)
        {
          return true;
        }
        else if (strcmp(command, "V") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Uq") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Vlim") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Ud") == 0)
        {
          return true;
        }
        else if (strcmp(command, "P") == 0)
        {
          return true;
        }

        else if (strcmp(command, "PD") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kpp") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kpv") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kpiq") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kpid") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kiv") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kiiq") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kiid") == 0)
        {
          return true;
        }
        else if (strcmp(command, "KP") == 0)
        {
          return true;
        }
        else if (strcmp(command, "KD") == 0)
        {
          return true;
        }
        else if (strcmp(command, "PDV") == 0)
        {
          return true;
        }
        else if (strcmp(command, "PDI") == 0)
        {
          return true;
        }
        else if (strcmp(command, "R") == 0)
        {
          return true;
        }
        else if (strcmp(command, "L") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kt") == 0)
        {
          return true;
        }
        else if (strcmp(command, "KV") == 0)
        {
          return true;
        }
        else if (strcmp(command, "flux") == 0)
        {
          return true;
        }
        else if (strcmp(command, "zz") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Cal") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Save") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Reset") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Ilim") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Dir") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Temp") == 0)
        {
          return true;
        }

        else if (strcmp(command, "Magnet") == 0)
        {
          return true;
        }

        else if (strcmp(command, "Term") == 0)
        {
          return true;
        }

        else if (strcmp(command, "Mode") == 0)
        {
          return true;
        }

        else if (strcmp(command, "Vbus") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Vmin") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Vmax") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Error") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Clear") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Default") == 0)
        {
          return true;
        }
        else if (strcmp(command, "SIN") == 0)
        {
          return true;
        }
        else if (strcmp(command, "SPWM") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Iabc") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Close") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Cbw") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Resv") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Calpwr") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Openv") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Opens") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Vp") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Calibrated") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Param") == 0)
        {
          return true;
        }
        else if (strcmp(command, "PP") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Openloop") == 0)
        {
          return true;
        }
        else if (strcmp(command, "LED") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Cyc") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Cyca") == 0)
        {
          return true;
        }
        else if (strcmp(command, "CANID") == 0)
        {
          return true;
        }
        else if (strcmp(command, "SERNUM") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Info") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Help") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Vlimit") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Phase") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Activate") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Default") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripper") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripcal") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Grippos") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripvel") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripcur") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripact") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripstop") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripinfo") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Rstint") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Pullconfig") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Tlim") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Calangle") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Offset") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Iqoffset") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Brake") == 0)
        {
          return true;
        }

        else
        {
          // Unknown command
          lastCommandUnknown = true;
          resetCommandAndArgument(command, argument);
          return false;
        }
      }
      else
      {
        // Combined length of command and argument exceeds 20 characters
        lastCommandUnknown = true;
        resetCommandAndArgument(command, argument);
        return false;
      }
    }

    else if (sscanf(buffer, "#%19s", command) == 1)
    {
      if (strlen(command) <= 20)
      {
        // Dispatch based on the parsed command
        if (strcmp(command, "Iq") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Id") == 0)
        {
          return true;
        }
        else if (strcmp(command, "V") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Uq") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Ud") == 0)
        {
          return true;
        }
        else if (strcmp(command, "P") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Vlim") == 0)
        {
          return true;
        }
        else if (strcmp(command, "PD") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kpp") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kpv") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kpiq") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kpid") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kiv") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kiiq") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kiid") == 0)
        {
          return true;
        }
        else if (strcmp(command, "KP") == 0)
        {
          return true;
        }
        else if (strcmp(command, "KD") == 0)
        {
          return true;
        }
        else if (strcmp(command, "PDV") == 0)
        {
          return true;
        }
        else if (strcmp(command, "PDI") == 0)
        {
          return true;
        }
        else if (strcmp(command, "R") == 0)
        {
          return true;
        }
        else if (strcmp(command, "L") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Kt") == 0)
        {
          return true;
        }
        else if (strcmp(command, "flux") == 0)
        {
          return true;
        }
        else if (strcmp(command, "KV") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Idle") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Cal") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Save") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Reset") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Ilim") == 0)
        {
          return true;
        }

        else if (strcmp(command, "Dir") == 0)
        {
          return true;
        }

        else if (strcmp(command, "Temp") == 0)
        {
          return true;
        }

        else if (strcmp(command, "Magnet") == 0)
        {
          return true;
        }

        else if (strcmp(command, "Term") == 0)
        {
          return true;
        }

        else if (strcmp(command, "Mode") == 0)
        {
          return true;
        }

        else if (strcmp(command, "Vbus") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Vmin") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Vmax") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Error") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Clear") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Default") == 0)
        {
          return true;
        }
        else if (strcmp(command, "SIN") == 0)
        {
          return true;
        }
        else if (strcmp(command, "SPWM") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Iabc") == 0)
        {
          return true;
        }
        else if (strcmp(command, "EN") == 0)
        {
          return true;
        }
        else if (strcmp(command, "PWM") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Cbw") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Resv") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Calpwr") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Openv") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Opens") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Vp") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Close") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Calibrated") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Param") == 0)
        {
          return true;
        }
        else if (strcmp(command, "PP") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Openloop") == 0)
        {
          return true;
        }
        else if (strcmp(command, "LED") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Cyc") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Cyca") == 0)
        {
          return true;
        }
        else if (strcmp(command, "CANID") == 0)
        {
          return true;
        }
        else if (strcmp(command, "SERNUM") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Info") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Help") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Vlimit") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Phase") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Activate") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Default") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripper") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripcal") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Grippos") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripvel") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripcur") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripact") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripstop") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Gripinfo") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Rstint") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Pullconfig") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Tlim") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Calangle") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Offset") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Iqoffset") == 0)
        {
          return true;
        }
        else if (strcmp(command, "Brake") == 0)
        {
          return true;
        }

        else
        {
          // Unknown command
          lastCommandUnknown = true;
          resetCommandAndArgument(command, argument);
          return false;
        }
      }
      else
      {
        // Command length exceeds 20 characters
        lastCommandUnknown = true;
        resetCommandAndArgument(command, argument);
        return false;
      }
    }
    else
    {
      // Invalid command or argument
      lastCommandUnknown = true;
      resetCommandAndArgument(command, argument);
      return false;
    }
  }
  return false; // No valid packet yet
}

bool SerialPacketParser::handleIq(char *argument)
{
  // Your code to handle the "start" command
  // Use 'argument' for any additional data related to this command

  return true; // Successfully handled
}

void SerialPacketParser::resetCommandAndArgument(char *command, char *argument)
{
  // Reset the command and argument arrays
  memset(command, 0, sizeof(command));
  memset(argument, 0, sizeof(argument));
}