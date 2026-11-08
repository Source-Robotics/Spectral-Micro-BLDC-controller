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
#include "utils.h"

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
    // "#Command argument": the command and argument words, each in its own 20-byte buffer
    const char *rest = (buffer[0] == '#') ? next_word(buffer + 1, command, 20) : NULL;
    if (rest != NULL && next_word(rest, argument, 20) != NULL)
    {
      // Dispatch based on the parsed command. Command and argument each have their own 20-byte buffer
      // and %19s already caps each at 19 characters; the combined limit was 20, which refused a full
      // "#WdFb 10000,-1200,2000,3", so it is now the sum of the two.
      if (strlen(command) + strlen(argument) <= 38)
      {
        // UART_protocol is the one place that knows the commands; it answers "Unknown command" for the rest
        // (batch 23: the list kept here as well was about 4 KB and had to be kept in step by hand)
        return true;
      }
      else
      {
        // Combined length of command and argument exceeds 20 characters
        lastCommandUnknown = true;
        resetCommandAndArgument(command, argument);
        return false;
      }
    }

    else if (rest != NULL) // "#Command" alone
    {
      if (strlen(command) <= 20)
      {
        return true; // as above: UART_protocol knows the commands
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