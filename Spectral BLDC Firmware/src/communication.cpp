/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    communication.cpp
 * @brief   This file provides code for our UART CLI
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

#include "communication.h"
#include "cogging.h"
#include "cogcal.h"
#include "encal.h"
#include "sound.h"
#include "communication_CAN.h" // Param_get, Param_set, Param_is_float (#Par)
void Uart_param_print(Stream &Serialport, uint8_t n, bool say_unknown);
void Uart_features_print(Stream &Serialport);
#include "bootloader_config.h"

/// Hexadecimal with its leading zeros.
static void Print_hex(Stream &Serialport, uint32_t value, int digits)
{
    for (int shift = (digits - 1) * 4; shift >= 0; shift -= 4)
        Serialport.print((unsigned int)(value >> shift) & 15u, HEX);
}

SerialPacketParser parser;
char command[20];
char argument[20];

/// @todo Cyclic ?
/// @todo Cascade commands
/*
 */

/// @param Serialport Serial port we want to use
void UART_protocol(Stream &Serialport)
{
    /*
       CLI INTERFACE
      */
    while (Serialport.available())
    {
        char c = Serialport.read();

        if (parser.parse(c, command, argument))
        {

            // Set Iq setpoint
            // Switches actuator to current control mode
            if (strcmp(command, "Iq") == 0)
            {

                if (strlen(argument) != 0)
                {
                    if (PID.Reset_integral_accumulator == 1)
                    {
                        PID.V_errSum = 0;
                        PID.Iq_errSum = 0;
                        PID.Id_errSum = 0;
                    }
                    PID.Iq_setpoint = parse_float(argument);
                    controller.Controller_mode = 3;
                    if (controller.reset_pin_state == 0 && controller.sleep_pin_state == 0)
                    {
                        digitalWriteFast(SLEEP, HIGH);
                        digitalWriteFast(RESET, HIGH);
                        controller.reset_pin_state = 1;
                        controller.sleep_pin_state = 1;
                    }
                }

                Serialport.print("Iq ");
                Serialport.println(FOC.Iq);
            }

            // Set Id setpoint
            else if (strcmp(command, "Id") == 0)
            {

                if (strlen(argument) != 0)
                {
                    PID.Id_setpoint = parse_float(argument);
                }

                Serialport.print("Id ");
                Serialport.println(FOC.Id);
            }

            // Get/Set voltage limit
            else if (strcmp(command, "Vlim") == 0)
            {

                if (strlen(argument) != 0)
                {
                    PID.Voltage_limit = atoi(argument);
                }

                Serialport.print("Vlim ");
                if (PID.Voltage_limit == 0)
                {
                    Serialport.println(controller.VBUS_mV);
                }
                else
                {
                    Serialport.println(PID.Voltage_limit);
                }
            }

            else if (strcmp(command, "Uq") == 0)
            {

                if (strlen(argument) != 0)
                {

                    if (PID.Reset_integral_accumulator == 1)
                    {
                        PID.V_errSum = 0;
                        PID.Iq_errSum = 0;
                        PID.Id_errSum = 0;
                    }

                    PID.Feedforward_current = 0;
                    PID.Uq_setpoint = atoi(argument);
                    controller.Controller_mode = 8;
                    if (controller.reset_pin_state == 0 && controller.sleep_pin_state == 0)
                    {
                        digitalWriteFast(SLEEP, HIGH);
                        digitalWriteFast(RESET, HIGH);
                        controller.reset_pin_state = 1;
                        controller.sleep_pin_state = 1;
                    }
                }

                Serialport.print("Uq ");
                Serialport.println(PID.Uq_setpoint);
            }

            // Set Ud voltage setpoint
            // Does not place motor in the voltage torque mode
            else if (strcmp(command, "Ud") == 0)
            {

                if (strlen(argument) != 0)
                {

                    if (PID.Reset_integral_accumulator == 1)
                    {
                        PID.V_errSum = 0;
                        PID.Iq_errSum = 0;
                        PID.Id_errSum = 0;
                    }

                    PID.Feedforward_current = 0;
                    PID.Ud_setpoint = atoi(argument);
                    // controller.Controller_mode = 6;
                    if (controller.reset_pin_state == 0 && controller.sleep_pin_state == 0)
                    {
                        digitalWriteFast(SLEEP, HIGH);
                        digitalWriteFast(RESET, HIGH);
                        controller.reset_pin_state = 1;
                        controller.sleep_pin_state = 1;
                    }
                }

                Serialport.print("Ud ");
                Serialport.println(PID.Ud_setpoint);
            }

            // Set Velocity setpoint
            // Switches actuator to velocity mode
            else if (strcmp(command, "V") == 0)
            {

                if (strlen(argument) != 0)
                {

                    if (PID.Reset_integral_accumulator == 1)
                    {
                        PID.V_errSum = 0;
                        PID.Iq_errSum = 0;
                        PID.Id_errSum = 0;
                    }

                    PID.Feedforward_current = 0;
                    PID.Velocity_setpoint = atoi(argument);
                    controller.Controller_mode = 2;
                    if (controller.reset_pin_state == 0 && controller.sleep_pin_state == 0)
                    {
                        digitalWriteFast(SLEEP, HIGH);
                        digitalWriteFast(RESET, HIGH);
                        controller.reset_pin_state = 1;
                        controller.sleep_pin_state = 1;
                    }
                }

                Serialport.print("V ");
                Serialport.println(controller.Velocity_Filter);
            }

            // Get/Set clamp velocity integrator to this value v
            else if (strcmp(command, "Vlimit") == 0)
            {

                if (strlen(argument) != 0)
                {
                    PID.Velocity_limit = abs(atoi(argument)); // a negative limit turned the speed clamp round
                }

                Serialport.print("Vlimit ");
                Serialport.println(PID.Velocity_limit);
            }

            // Set Position setpoint
            // Switches actuator to position mode
            else if (strcmp(command, "P") == 0)
            {

                if (strlen(argument) != 0)
                {

                    if (PID.Reset_integral_accumulator == 1)
                    {
                        PID.V_errSum = 0;
                        PID.Iq_errSum = 0;
                        PID.Id_errSum = 0;
                    }

                    PID.Feedforward_speed = 0;
                    PID.Feedforward_current = 0;
                    PID.Position_setpoint = atoi(argument);
                    controller.Controller_mode = 1;
                    if (controller.reset_pin_state == 0 && controller.sleep_pin_state == 0)
                    {
                        digitalWriteFast(SLEEP, HIGH);
                        digitalWriteFast(RESET, HIGH);
                        controller.reset_pin_state = 1;
                        controller.sleep_pin_state = 1;
                    }
                }

                Serialport.print("P ");
                Serialport.println(controller.Position_Ticks);
            }

            // Switches actuator to PD mode and sets/gets position setpoint
            else if (strcmp(command, "PD") == 0)
            {
                if (strlen(argument) != 0)
                {
                    if (PID.Reset_integral_accumulator == 1)
                    {
                        PID.V_errSum = 0;
                        PID.Iq_errSum = 0;
                        PID.Id_errSum = 0;
                    }
                    if (controller.reset_pin_state == 0 && controller.sleep_pin_state == 0)
                    {
                        digitalWriteFast(SLEEP, HIGH);
                        digitalWriteFast(RESET, HIGH);
                        controller.reset_pin_state = 1;
                        controller.sleep_pin_state = 1;
                    }
                    controller.Controller_mode = 4;

                    PID.Position_setpoint = atoi(argument);
                }
                Serialport.print("PD ");
                Serialport.println(controller.Position_Ticks);
            }

            // Set get velocity setpoint when in PD mode
            else if (strcmp(command, "PDV") == 0)
            {
                if (strlen(argument) != 0)
                {
                    PID.Velocity_setpoint = atoi(argument);
                }
                Serialport.print("PDV ");
                Serialport.println(PID.Velocity_setpoint);
            }

            // Set get Current setpoint when in PD mode
            else if (strcmp(command, "PDI") == 0)
            {

                if (strlen(argument) != 0)
                {
                    PID.Feedforward_current = atoi(argument);
                }
                Serialport.print("PDI ");
                Serialport.println(PID.Feedforward_current);
            }

            // Set/Get Kpp (Position loop Kp gain)
            else if (strcmp(command, "Kpp") == 0)
            {
                if (strlen(argument) != 0)
                {
                    PID.Kp_p = parse_float(argument);
                }
                Serialport.print("Kpp ");
                Serialport.println(PID.Kp_p, 5);
            }

            // Set/Get KP (PD loop)
            else if (strcmp(command, "KP") == 0)
            {
                if (strlen(argument) != 0)
                {
                    PID.KP = parse_float(argument);
                }
                Serialport.print("KP ");
                Serialport.println(PID.KP, 6);
            }

            // Set/Get KD (PD loop)
            else if (strcmp(command, "KD") == 0)
            {
                if (strlen(argument) != 0)
                {
                    PID.KD = parse_float(argument);
                }
                Serialport.print("KD ");
                Serialport.println(PID.KD, 6);
            }

            // Set/Get Kpv (Velocity loop Kp gain)
            else if (strcmp(command, "Kpv") == 0)
            {

                if (strlen(argument) != 0)
                {
                    PID.Kp_v = parse_float(argument);
                }
                Serialport.print("Kpv ");
                Serialport.println(PID.Kp_v, 5);
            }

            // Set/Get Kpiq (Iq current loop Kp gain)
            // It will also set Kpid
            else if (strcmp(command, "Kpiq") == 0)
            {

                if (strlen(argument) != 0)
                {
                    PID.Kp_iq = parse_float(argument);
                    PID.Kp_id = PID.Kp_iq;
                }
                Serialport.print("Kpiq ");
                Serialport.println(PID.Kp_iq, 5);
            }

            // Set/Get Kpid (Id current loop Kp gain)
            else if (strcmp(command, "Kpid") == 0)
            {

                if (strlen(argument) != 0)
                {
                    PID.Kp_id = parse_float(argument);
                }
                Serialport.print("Kpid ");
                Serialport.println(PID.Kp_id, 5);
            }

            // Set/Get Kpiv (Velocity loop integrator gain)
            else if (strcmp(command, "Kiv") == 0)
            {

                if (strlen(argument) != 0)
                {
                    PID.Ki_v = parse_float(argument);
                }
                Serialport.print("Kiv ");
                Serialport.println(PID.Ki_v, 5);
            }

            // Set/Get Kiiq (Iq loop integrator gain)
            // It will also set Kiid
            else if (strcmp(command, "Kiiq") == 0)
            {

                if (strlen(argument) != 0)
                {
                    PID.Ki_iq = parse_float(argument);
                    PID.Ki_id = PID.Ki_iq;
                }
                Serialport.print("Kiiq ");
                Serialport.println(PID.Ki_iq, 5);
            }

            // Set/Get Kiid (Id loop integrator gain)
            else if (strcmp(command, "Kiid") == 0)
            {

                if (strlen(argument) != 0)
                {
                    PID.Ki_id = parse_float(argument);
                }
                Serialport.print("Kiid ");
                Serialport.println(PID.Ki_id, 5);
            }

            // Set/Get Resistance
            else if (strcmp(command, "R") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.Resistance = parse_float(argument);
                }
                Serialport.print("R ");
                Serialport.println(controller.Resistance, 5);
            }

            // Set/Get Inductance
            else if (strcmp(command, "L") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.Inductance = parse_float(argument);
                }
                Serialport.print("L ");
                Serialport.println(controller.Inductance, 5);
            }

            // Set/Get Kt (Torque constant)
            else if (strcmp(command, "Kt") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.Kt = parse_float(argument);
                }
                Serialport.print("Kt ");
                Serialport.println(controller.Kt, 5);
            }

            // Set/Get KV (motor velocity constant)
            else if (strcmp(command, "KV") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.KV = parse_float(argument);
                }
                Serialport.print("KV ");
                Serialport.println(controller.KV, 5);
            }

            // Set/Get Max motor voltage
            else if (strcmp(command, "Vmax") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.Max_Vbus = atoi(argument);
                }
                Serialport.print("Vmax in mV: ");
                Serialport.println(controller.Max_Vbus);
            }

            // Set/Get Flux linkage
            else if (strcmp(command, "Flux") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.flux_linkage = parse_float(argument);
                }
                Serialport.print("Flux ");
                Serialport.println(controller.flux_linkage, 5);
            }

            // Go to idle state
            else if (strcmp(command, "Idle") == 0)
            {

                controller.Controller_mode = 0;
                Serialport.println("Idle");
            }

            // E-stop over UART, as the CAN E-stop frame: the E-stop error is latched, the motor is released and a
            // calibration that is running ends (#Idle does not end one). #Clear clears it. Until 2026-11 only CAN
            // could do this; a host on UART had to reset the drive to stop a calibration.
            else if (strcmp(command, "Estop") == 0)
            {
                controller.ESTOP_error = 1;
                Serialport.println("E-stop");
            }

            // Set/Get idle-mode phase behavior: 0 = coast (driver disabled, Hi-Z),
            // 1 = brake (driver stays enabled, zero-voltage PWM vector shorts the phases).
            // Only takes effect once idle (Controller_mode == 0) -- send Idle first/after.
            else if (strcmp(command, "Brake") == 0)
            {
                if (strlen(argument) != 0)
                {
                    controller.brake_coast = (atoi(argument) != 0) ? 1 : 0;
                }
                Serialport.print("Brake ");
                Serialport.println(controller.brake_coast);
            }

            // Enter calibration mode
            // #Cal: the calibration, then always the encoder table and the anti-cogging, then save;
            // #Calonly: the calibration alone
            else if ((strcmp(command, "Cal") == 0 || strcmp(command, "Calonly") == 0) && controller.ESTOP_error)
            {
                // it would end on its first loop, and a failed calibration marks the drive as not calibrated
                Serialport.println("Cal: not started, the E-stop is latched (#Clear first)");
            }
            else if (strcmp(command, "Cal") == 0 || strcmp(command, "Calonly") == 0)
            {
                Encal_skip_next_after_calibration(strcmp(command, "Calonly") == 0);
                if (controller.reset_pin_state == 0 && controller.sleep_pin_state == 0)
                {
                    digitalWriteFast(SLEEP, HIGH);
                    digitalWriteFast(RESET, HIGH);
                    controller.reset_pin_state = 1;
                    controller.sleep_pin_state = 1;
                }
                controller.Calibration = 1;
                if (Cal_saved_current_limit <= 0.0f)
                    Cal_saved_current_limit = PID.Iq_current_limit; // put back at the end (batch 22d)
                controller.Spin_confirm_delta = 0; // diagnostic: clear any stale value from a prior #Cal attempt
                Ticker_detach(TIM3);
                controller.Controller_mode = 0; // also ends an encoder-table or anti-cogging run that was turning the motor
                // Nothing carried into the calibration from a mode that was driving: its interrupt uses the output
                // rotation and the tone of the last control loop as they stand, and the current integrals.
                FOC.delay_c = 1.0f;
                FOC.delay_s = 0.0f;
                FOC.sound_d = 0.0f;
                FOC.sound_q = 0.0f;
                PID.Iq_errSum = 0.0f;
                PID.Id_errSum = 0.0f;
                Ticker_init(TIM3, LOOP_FREQ, Update_IT_callback_calib);
            }

            // set get theta_offset
            else if (strcmp(command, "Offset") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.theta_offset = parse_float(argument);
                }
                Serialport.print("Offset ");
                Serialport.println(controller.theta_offset, 5);
            }

            // set get Iqoffset that will be used to search angle offset
            else if (strcmp(command, "Iqoffset") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.calibration_offset_current = atoi(argument);
                }
                Serialport.print("Iq Offset current: ");
                Serialport.println(controller.calibration_offset_current);
            }

            // Check position sensor magnet warrning
            else if (strcmp(command, "Magnet") == 0)
            {
                Serialport.print("Magnet ");
                if (controller.Magnet_warrning == 0)
                {
                    Serialport.println("Good");
                }
                else
                {
                    Serialport.println("Bad");
                }
            }

            // Save config
            else if (strcmp(command, "Save") == 0)
            {
                Write_config();
                Serialport.println("Save");
            }

            // Reset MCU
            else if (strcmp(command, "Reset") == 0)
            {
                Bootloader_ResetIntoBootloader();
            }

            // Set Iq current limit
            // If bigger than what driver FETS are capable; clamp it
            else if (strcmp(command, "Ilim") == 0)
            {
                if (strlen(argument) != 0)
                {
                    int var = atoi(argument);
                    if (abs(var) > MAX_DRIVE_CURRENT)
                    {
                        PID.Iq_current_limit = MAX_DRIVE_CURRENT;
                    }
                    else
                    {
                        PID.Iq_current_limit = abs(var); // a negative limit turned the current clamps round
                    }
                }
                Serialport.print("Ilim ");
                Serialport.println(PID.Iq_current_limit);
            }

            // Set new max temperature limit
            else if (strcmp(command, "Tlim") == 0)
            {
                if (strlen(argument) != 0)
                {
                    int var = atoi(argument);
                    controller.Max_temperature = var;
                }
                Serialport.print("Temp limit ");
                Serialport.println(controller.Max_temperature);
            }

            // Set/Get Dir
            else if (strcmp(command, "Dir") == 0)
            {
                if (strlen(argument) != 0)
                {
                    int var = atoi(argument);
                    if (abs(var) >= 1)
                    {
                        controller.DIR_ = 1;
                    }
                    else
                    {
                        controller.DIR_ = 0;
                    }
                }

                Serialport.print("Dir ");
                Serialport.println(controller.DIR_);
            }

            // Get phase order
            else if (strcmp(command, "Phase") == 0)
            {
                Serialport.print("Phase order is: ");
                Serialport.println(controller.Phase_order);
            }

            // Load default config to EEPROM
            else if (strcmp(command, "Default") == 0)
            {
                Serialport.print("Loaded default config!");
                Set_Default_config();
                // The defaults carry a CAN id; the hardware filter follows it.
                if (controller.CAN_init_error == 0)
                    CANSetNodeFilter(controller.CAN_ID);
            }

            // Get termistor temperature in degrees
            else if (strcmp(command, "Temp") == 0)
            {
                Serialport.print("Temp ");
                if (controller.Thermistor_on_off == 1)
                {
                    Serialport.println(controller.TEMP_DEG);
                }
                else
                {
                    Serialport.println("Off");
                }
            }

            // Enable/disable termistor mesurement 0 is off, 1 is on
            else if (strcmp(command, "Term") == 0)
            {
                if (strlen(argument) != 0)
                {
                    int var = atoi(argument);
                    if (var >= 1)
                    {
                        controller.Thermistor_on_off = 1;
                    }
                    else
                    {
                        controller.Thermistor_on_off = 0;
                    }
                }
                Serialport.print("Term ");
                Serialport.println(controller.Thermistor_on_off);
            }

            // Print controller mode
            else if (strcmp(command, "Mode") == 0)
            {

                if (controller.Controller_mode == 0)
                {
                    Serialport.println("Idle mode ");
                }
                else if (controller.Controller_mode == 1)
                {
                    Serialport.println("Position control ");
                }
                else if (controller.Controller_mode == 2)
                {
                    Serialport.println("Speed control ");
                }
                else if (controller.Controller_mode == 3)
                {
                    Serialport.println("Current control ");
                }
                else if (controller.Controller_mode == 4)
                {
                    Serialport.println("PD control ");
                }
                else if (controller.Controller_mode == 5)
                {
                    Serialport.println("Open loop");
                }
                else if (controller.Controller_mode == 6)
                {
                    Serialport.println("Gripper mode");
                }
                else if (controller.Controller_mode == 7)
                {
                    Serialport.println("Gripper calib mode");
                }
            }

            // Print Vbus in mA
            else if (strcmp(command, "Vbus") == 0)
            {
                Serialport.print("Vbus ");
                Serialport.println(controller.VBUS_mV);
            }

            // Print if there are any errors
            else if (strcmp(command, "Error") == 0)
            {
                if (controller.Error == 1)
                {
                    Serialport.println("In error state!");
                }
                else
                {
                    Serialport.println("No errors!");
                }
                if (digitalReadFast(FAULT) == 0)
                {
                    Serialport.println("FAULT pin is LOW: the gate driver reports a fault");
                }
                if (controller.Driver_error == 1)
                {
                    Serialport.println("Drv error ");
                }
                if (controller.temperature_error == 1)
                {
                    Serialport.println("Temp error ");
                }
                if (controller.encoder_error == 1)
                {
                    Serialport.println("Encoder error ");
                }
                if (controller.Vbus_error == 1)
                {
                    Serialport.println("Vbus error ");
                }
                if (controller.Velocity_error == 1)
                {
                    Serialport.println("Velocity error ");
                }
                if (controller.Current_error == 1)
                {
                    Serialport.println("Current error ");
                }
                if (controller.Calibrated == 0)
                {
                    Serialport.println("Not calibrated");
                }
                if (controller.ESTOP_error == 1)
                {
                    Serialport.println("Received ESTOP command");
                }
                if (controller.Watchdog_error == 1)
                {
                    Serialport.println("Watchdog error");
                }
                if (controller.wrong_product == 1)
                {
                    Serialport.println("Wrong firmware for this board (product lock)");
                }
                // if (controller.Activated == 1)
                //{
                //  Serialport.println("Controller is activated");
                //}
                else
                {
                    // Serialport.println("Controller is NOT activated");
                }
            }

            // Clear All errors and set mode to IDLE
            // Will not clear calibration error. If motor is not calibrated you will
            // need to set controller.Calibrated to 1. with #Calibrated 1 command
            else if (strcmp(command, "Clear") == 0)
            {
                digitalWriteFast(SLEEP, HIGH);
                digitalWriteFast(RESET, HIGH);
                controller.encoder_error = 0;
                controller.temperature_error = 0;
                controller.Vbus_error = 0;
                controller.Driver_error = 0;
                controller.Velocity_error = 0;
                controller.Error = 0;
                controller.ESTOP_error = 0;
                controller.Watchdog_error = 0;
                controller.Controller_mode = 0;
                Gripper.estop_error = 0;
                Serialport.println("Clear errors");
            }

            // Print phase currents
            else if (strcmp(command, "Iabc") == 0)
            {
                Serialport.print("Ia ");
                Serialport.println(controller.Sense1_mA);
                Serialport.print("Ib ");
                Serialport.println(controller.Sense2_mA);
                Serialport.print("Ic ");
                Serialport.println(controller.Sense3_mA);
            }
            // Enter closed loop mode, idle state
            // #Close while a calibration runs: end it the way an E-stop or its own failure does (its steps start
            // over next time, the current and voltage limits are put back, the bridge is off). Taking the interrupt
            // away from it, as #Close does below, left it half-done: the next #Cal went on from the old step and the
            // calibration's 1600 mA limit could be saved.
            else if (strcmp(command, "Close") == 0 && controller.Calibration == 1)
            {
                controller.Calib_error = 1;
                Serialport.println("Close: the calibration is stopped");
            }
            else if (strcmp(command, "Close") == 0)
            {
                digitalWriteFast(EN1, HIGH);
                digitalWriteFast(EN2, HIGH);
                digitalWriteFast(EN3, HIGH);
                digitalWriteFast(RESET, HIGH);
                digitalWriteFast(SLEEP, HIGH);
                controller.Controller_mode = 0;
                pwm_set(PWM_CH1, 0, 13);
                pwm_set(PWM_CH2, 0, 13);
                pwm_set(PWM_CH3, 0, 13);
                Ticker_detach(TIM3);
                Ticker_init(TIM3, LOOP_FREQ, IT_callback);
            }

            // Print PWM values
            else if (strcmp(command, "PWM") == 0)
            {
                Serialport.print("PWM1 ");
                Serialport.println(FOC.PWM1);
                Serialport.print("PWM2 ");
                Serialport.println(FOC.PWM2);
                Serialport.print("PWM3 ");
                Serialport.println(FOC.PWM3);
            }

            // Set/Get Current loop bandwidth
            else if (strcmp(command, "Cbw") == 0)
            {

                if (strlen(argument) != 0)
                {
                    float bw = parse_float(argument);
                    if (bw >= CURRENT_BANDWIDTH_MIN && bw <= CURRENT_BANDWIDTH_MAX)
                    {
                        float before = controller.current_control_bandwidth;
                        controller.current_control_bandwidth = bw;
                        if (!Current_gains_from_RL()) // the gains follow at once, as with CAN parameter 49
                            controller.current_control_bandwidth = before;
                    }
                }
                Serialport.print("Current loop bandwidth: ");
                Serialport.println(controller.current_control_bandwidth, 5);
                Serialport.print("Kp_iq = ");
                Serialport.print(PID.Kp_iq, 5);
                Serialport.print(", Ki_iq = ");
                Serialport.println(PID.Ki_iq, 5);
            }

            // Set/get voltage used when looking for resistance value
            else if (strcmp(command, "Resv") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.Resistance_calc_voltage = atoi(argument);
                }
                Serialport.print("Resistance voltage: ");
                Serialport.println(controller.Resistance_calc_voltage);
            }

            // Set/get max power that can be used during Inductance and resistance search
            else if (strcmp(command, "Calpwr") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.Calibration_power = parse_float(argument);
                }
                Serialport.print("Max Power dissipation: ");
                Serialport.println(controller.Calibration_power, 5);
            }

            // Set/get open loop calibration voltage
            else if (strcmp(command, "Openv") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.Open_loop_voltage = atoi(argument);
                }
                Serialport.print("Open loop voltage: ");
                Serialport.println(controller.Open_loop_voltage);
            }

            // Set/get open loop calibration speed
            else if (strcmp(command, "Opens") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.Open_loop_speed = parse_float(argument);
                }
                Serialport.print("Open loop Speed: ");
                Serialport.println(controller.Open_loop_speed, 5);
            }

            // Set/get Phase order search voltage
            else if (strcmp(command, "Vp") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.Phase_voltage = atoi(argument);
                }
                Serialport.print("Phase search voltage: ");
                Serialport.println(controller.Phase_voltage);
            }

            // Print a summary of available commands
            else if (strcmp(command, "Help") == 0)
            {
                Serialport.println("--- Spectral BLDC command reference ---");
                Serialport.println("Usage: #<Command> [value]  (omit value to query current)");
                Serialport.println("");
                Serialport.println("Motion / control mode:");
                Serialport.println("  Iq <mA>      Torque mode, set Iq setpoint");
                Serialport.println("  Id <mA>      Set Id setpoint");
                Serialport.println("  V <ticks/s>  Velocity mode setpoint");
                Serialport.println("  P <ticks>    Position mode setpoint");
                Serialport.println("  PD <ticks>   Impedance (PD) mode setpoint");
                Serialport.println("  Uq/Ud <mV>   Voltage-torque mode");
                Serialport.println("  Idle         Go to idle");
                Serialport.println("  Brake <0|1>  Idle phase behavior: 0=coast (Hi-Z), 1=brake (short)");
                Serialport.println("  Openloop <e> Open-loop speed mode (electrical speed)");
                Serialport.println("");
                Serialport.println("Calibration:");
                Serialport.println("  Cal          Calibration, encoder table, anti-cogging, then save");
                Serialport.println("  Calonly      The calibration alone");
                Serialport.println("  R/L <ohm/H>  Set phase resistance/inductance before Cal");
                Serialport.println("  PP <n>       Set pole pairs before Cal");
                Serialport.println("  Par [n[,v]]  CAN parameter n: read it, or set it to v (no n: all)");
                Serialport.println("  Tone f,mA,ms A tone through the motor (#Tone off stops it)");
                Serialport.println("  Save         Save current config to EEPROM");
                Serialport.println("  Default      Reset config to factory defaults");
                Serialport.println("");
                Serialport.println("Gains (get/set with #<name> <value>):");
                Serialport.println("  Kpp Kpv Kiv Kpiq Kiiq Kpid Kiid KP KD Vlim Ilim Vlimit");
                Serialport.println("");
                Serialport.println("Gripper mode:");
                Serialport.println("  Gripper      Enter gripper mode");
                Serialport.println("  Gripcal      Calibrate gripper");
                Serialport.println("  Grippos/Gripvel/Gripcur   Gripper setpoints");
                Serialport.println("  Gripinfo     Gripper status");
                Serialport.println("");
                Serialport.println("Status:");
                Serialport.println("  Info         Full driver/motor status dump");
                Serialport.println("  Error        Decoded error flags");
                Serialport.println("  Clear        Clear all errors, go idle");
                Serialport.println("  CANID <id>   Get/set CAN node ID");
                Serialport.println("  Tool <id>    Get/set tool id (0 none, 1 SSG-48, 11-33 MSG)");
                Serialport.println("  WdFb t,i,v,a Get/set watchdog fallback (ms, mA, ticks/s, after)");
                Serialport.println("  Product      Board stamp; Product claim / Product clear");
                Serialport.println("");
                Serialport.println("See communication.cpp for the complete command list.");
            }

            // Print Information about motor driver and motor
            else if (strcmp(command, "Info") == 0)
            {

                Serialport.print("Hardware version: ");
                Serialport.println(controller.HARDWARE_VERSION);
                Serialport.print("Batch date: ");
                Serialport.println(controller.BATCH_DATE);
                Serialport.print("Software version: ");
                Serialport.println(controller.SOFTWARE_VERSION);
                Serialport.print("Build: ");
                Print_hex(Serialport, BUILD_COMMIT & 0x0FFFFFFFu, 7);
                Serialport.println((BUILD_COMMIT >> 28) & 1u ? "+" : "");
                Serialport.print("Board ID: ");
                Print_hex(Serialport, Board_id(), 8);
                Serialport.println();
                Serialport.print("Calibration ID: ");
                Print_hex(Serialport, controller.calibration_id, 8);
                Serialport.println();
                Serialport.print("ISR execution time: ");
                Serialport.print(controller.execution_time);
                Serialport.println(" us");
                Serialport.print("CAN ID is: ");
                Serialport.println(controller.CAN_ID);
                Serialport.print("CAN RX overruns: ");
                Serialport.println(CAN_rx_overruns);
                Serialport.print("CAN TX dropped: ");
                Serialport.println(CAN_tx_dropped);
                if (controller.wrong_product)
                {
                    Serialport.println("WRONG FIRMWARE: this board was calibrated by the other product. This firmware drives");
                    Serialport.println("nothing; flash the right firmware (or #Product claim if the board really is this product).");
                }
                Serialport.println("Product: Spectral Micro");
                Serialport.print("Current sense zero: ");
                Serialport.print(controller.sense1_offset);
                Serialport.print(" / ");
                Serialport.print(controller.sense2_offset);
                Serialport.println(controller.sense_zero_wait_vbus ? " (no motor supply at start-up: 2048 used; measured once the supply is there)" :
                                   controller.sense_zero_switching == SENSE_ZERO_SW_IN_USE ? " counts from 2048, measured with the bridge switching" :
                                   (controller.sense_zero_ok ? " counts from 2048, measured with the bridge off" : " (not measured or out of range: 2048 used)"));
                Serialport.print("Current sense zero with the bridge off: ");
                Serialport.print(controller.sense1_offset_off);
                Serialport.print(" / ");
                Serialport.print(controller.sense2_offset_off);
                Serialport.print("; switching zero: ");
                Serialport.println(controller.sense_zero_switching == SENSE_ZERO_SW_IN_USE ? "in use" :
                                   controller.sense_zero_switching == SENSE_ZERO_SW_REJECTED ? "rejected (implausible)" :
                                   controller.sense_zero_switching == SENSE_ZERO_SW_INTERRUPTED ? "interrupted (a command or an error came first)" :
                                   controller.sense_zero_switching == SENSE_ZERO_SW_RUNNING ? "measuring" :
                                   controller.sense_zero_switching == SENSE_ZERO_SW_MEASURED ? "measured, not in use" : "not run");
                Serialport.print("Tool ID: ");
                Serialport.println(controller.tool_id);
                Serialport.print("Watchdog: ");
                Serialport.print(controller.watchdog_time_ms);
                Serialport.print(" ms, action ");
                Serialport.print(controller.watchdog_action);
                Serialport.print(", fallback ");
                Serialport.print(controller.wd_fallback_time_ms);
                Serialport.print(" ms / ");
                Serialport.print(controller.wd_fallback_current);
                Serialport.print(" mA / ");
                Serialport.print(controller.wd_fallback_speed);
                Serialport.print(" ticks/s, then ");
                Serialport.println(controller.wd_after_action);
                Serialport.print("Settings block 2: ");
                Serialport.println(controller.eeprom_block2_ok ? "ok" : "not available (EEPROM has no second bank)");
                Serialport.print("Anti-cogging: ");
                Serialport.print(cogging.mode == COGGING_MODE_HARMONICS ? "harmonics" : (cogging.mode == COGGING_MODE_TABLE ? "table" : "off"));
                Serialport.print(", scale ");
                Serialport.print(cogging.scale);
                Serialport.println(Cogging_is_saved() ? " %, saved" : (controller.eeprom_block3_ok ? " %, not saved" : " %, block 3 not available"));
                if (controller.Calibrated == 0)
                {
                    Serialport.println("Not calibrated");
                }
                else
                {
                    Serialport.println("Calibrated");
                }

                if (controller.Thermistor_on_off == 0)
                {
                    Serialport.println("Termistor is disabled");
                }
                else
                {
                    Serialport.print("Temperature is:");
                    Serialport.print(controller.TEMP_DEG);
                    Serialport.println(" Degrees");
                }
                Serialport.print("Vbus voltage: ");
                Serialport.print(controller.VBUS_mV);
                Serialport.println(" mV");
                Serialport.print("Phase direction: ");
                Serialport.println(controller.DIR_);
                Serialport.print("Phase Order: ");
                Serialport.println(controller.Phase_order);
                Serialport.print("Commutation dir: ");
                Serialport.println(controller.commutation_dir);
                Serialport.print("Calib fwd/rev speed: ");
                Serialport.print(controller.Velocity_fwd);
                Serialport.print(" / ");
                Serialport.println(controller.Velocity_bwd);
                Serialport.print("Number of pole pairs: ");
                Serialport.println(controller.pole_pairs);
                Serialport.print("Single phase resistance: ");
                Serialport.println(controller.Resistance, 5);
                Serialport.print("Single phase Inductance: ");
                Serialport.println(controller.Inductance, 5);
                Serialport.print("Kt: ");
                Serialport.println(controller.Kt, 5);
                Serialport.print("KV: ");
                Serialport.println(controller.KV, 5);
                Uart_features_print(Serialport); // what is in force (batch 20)
            }

            // Set/get Am I a gripper?
            else if (strcmp(command, "Gripper") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.I_AM_GRIPPER = atoi(argument);
                }
                Serialport.print("Gripper: ");
                Serialport.println(controller.I_AM_GRIPPER);
            }

            // Set/get the tool id: what this drive is built into (0 none, 1 SSG-48, 11-33 MSG with
            // tens = rail 1/2/3 = 100/150/200 mm and units = stepper 1/2/3 = 21.5/40/60 mm, 200-254 custom).
            // Saved with #Save.
            else if (strcmp(command, "Tool") == 0)
            {
                if (strlen(argument) != 0)
                {
                    int temp_var = atoi(argument);
                    if (temp_var < 0 || temp_var > 255)
                        Serialport.println("Invalid tool id (0-255)");
                    else
                        controller.tool_id = temp_var;
                }
                Serialport.print("Tool ID: ");
                Serialport.println(controller.tool_id);
            }

            // The encoder table on the drive (encal.h): #Encal measures it (one shaft turn each way at 2048 ticks/s,
            // about 20 s; the shaft must be free), #Encal save also saves, #Encal all also identifies the anti-cogging
            // and saves both, #Encal stop aborts, #Encal on / off switches the correction, #Encal state shows it.
            // A tone through the motor (batch 22): #Tone f,mA[,ms] (ms left out: 500; 0: until #Tone off); #Tone off.
            // It plays while the drive holds or moves; from idle the drive first holds the position it is at.
            else if (strcmp(command, "Tone") == 0)
            {
                if (strlen(argument) == 0 || strcmp(argument, "off") == 0)
                {
                    Sound_stop();
                    Serialport.println("Tone off");
                }
                else
                {
                    const long f = atol(argument);
                    const char *c1 = strchr(argument, ',');
                    const long ma = c1 ? atol(c1 + 1) : 200;
                    const char *c2 = c1 ? strchr(c1 + 1, ',') : nullptr;
                    const long ms = c2 ? atol(c2 + 1) : 500;
                    if (controller.Controller_mode == 0 && controller.Calibrated && controller.Error == 0)
                    {
                        const uint32_t irq = __get_PRIMASK();
                        __disable_irq();
                        PID.Feedforward_speed = 0;
                        PID.Feedforward_current = 0;
                        PID.V_errSum = 0;
                        PID.Position_setpoint = controller.Position_Ticks;
                        controller.Controller_mode = 1;
                        if (controller.reset_pin_state == 0 && controller.sleep_pin_state == 0)
                        {
                            digitalWriteFast(SLEEP, HIGH);
                            digitalWriteFast(RESET, HIGH);
                            controller.reset_pin_state = 1;
                            controller.sleep_pin_state = 1;
                        }
                        __set_PRIMASK(irq);
                        Serialport.println("Holding the position while it plays (#Idle releases it)");
                    }
                    if (controller.Controller_mode >= 1 && controller.Controller_mode <= 4 && f > 0 && ma > 0 &&
                        Sound_tone(0, (uint16_t)constrain(f, 0, 65535), (uint16_t)constrain(ma, 0, 65535),
                                   (uint16_t)constrain(ms, 0, 65535)))
                    {
                        Serialport.print("Tone ");
                        Serialport.print(f);
                        Serialport.print(" Hz, ");
                        Serialport.print(ma);
                        Serialport.print(" mA, ");
                        if (ms)
                        {
                            Serialport.print(ms);
                            Serialport.println(" ms");
                        }
                        else
                            Serialport.println("until #Tone off");
                    }
                    else
                        Serialport.println("Tone: not played (calibrate first; 20 Hz up to half the loop rate; "
                                           "parameter 81 not 0)");
                }
            }
            // Any CAN parameter by number (batch 20): #Par lists all, #Par n reads one, #Par n,v sets it
            else if (strcmp(command, "Par") == 0)
            {
                if (strlen(argument) == 0)
                {
                    for (int n = 1; n < 256; n++)
                        Uart_param_print(Serialport, (uint8_t)n, false);
                }
                else
                {
                    const int n = atoi(argument);
                    const char *comma = strchr(argument, ',');
                    if (n < 1 || n > 255)
                        Serialport.println("Par: a number 1-255, then optionally ,value");
                    else
                    {
                        if (comma != nullptr)
                        {
                            int32_t v = 0;
                            if (Param_is_float((uint8_t)n))
                            {
                                const float f = parse_float(comma + 1);
                                memcpy(&v, &f, 4);
                            }
                            else
                                v = atol(comma + 1);
                            if (!Param_set((uint8_t)n, v))
                                Serialport.println("refused: out of range, read only, not now, or unknown");
                        }
                        Uart_param_print(Serialport, (uint8_t)n, true);
                    }
                }
            }
            else if (strcmp(command, "Encal") == 0)
            {
                // Only the words below do something: any other argument (a typo, "0") used to start the run and turn
                // the motor. on and off wait for a run to end: switched during one, the table came out wrong.
                const bool encal_turning = (Encal_state() == ENCAL_FORWARD || Encal_state() == ENCAL_BACKWARD);
                if (strcmp(argument, "stop") == 0)
                    Encal_abort();
                else if (strcmp(argument, "off") == 0)
                {
                    if (!encal_turning)
                        encoder.use = 0;
                }
                else if (strcmp(argument, "on") == 0)
                {
                    if (!encal_turning)
                        encoder.use = encoder.valid;
                }
                else if (strlen(argument) == 0 || strcmp(argument, "save") == 0 || strcmp(argument, "all") == 0)
                    Encal_start(strcmp(argument, "save") == 0, strcmp(argument, "all") == 0 ? 2 : 0);
                const uint8_t st = Encal_state();
                int32_t pp = 0, once = 0;
                Encal_param_get(PARAM_ENCAL_PP, &pp);
                Encal_param_get(PARAM_ENCAL_ONCE, &once);
                Serialport.print("Encoder table: ");
                Serialport.println(st == ENCAL_FORWARD ? "turning forward" : st == ENCAL_BACKWARD ? "turning backward" :
                                   st == ENCAL_DONE ? "done, in use" : st == ENCAL_FAILED_ABORTED ? "aborted" :
                                   st == ENCAL_FAILED_ERROR ? "failed: error or not calibrated" :
                                   st == ENCAL_FAILED_STUCK ? "failed: the rotor did not follow" :
                                   st == ENCAL_REJECTED ? "rejected: implausible" : "idle");
                Serialport.print(encoder.valid ? (encoder.use ? "in use" : "measured, not in use") : "not measured");
                Serialport.print("; error ");
                Serialport.print(pp / 100.0f, 2);
                Serialport.print(" deg peak to peak, ");
                Serialport.print(once / 100.0f, 2);
                Serialport.print(" deg once per turn; last offset move ");
                Serialport.print(encal_offset_x10 / 10.0f, 1);
                Serialport.println(" deg electrical");
            }

            // On-drive anti-cogging identification (cogcal.h): #Cogcal runs it, #Cogcal save also saves, #Cogcal stop
            // aborts, #Cogcal state shows it. One motor turn each way at the cogcal speed: the shaft must be free to turn.
            else if (strcmp(command, "Cogcal") == 0)
            {
                // Only these words do something: any other argument ("off", "0", a typo) used to start the run.
                if (strcmp(argument, "stop") == 0)
                {
                    Encal_abort(); // a combined run may still be in its encoder part
                    Cogcal_abort();
                }
                else if (strlen(argument) == 0 || strcmp(argument, "save") == 0 || strcmp(argument, "trim") == 0)
                    Cogcal_start(strcmp(argument, "save") == 0, strcmp(argument, "trim") == 0);
                const uint8_t st = Cogcal_state();
                Serialport.print("Anti-cogging identification: ");
                Serialport.println(st == COGCAL_FORWARD ? "turning forward" : st == COGCAL_BACKWARD ? "turning backward" :
                                   st == COGCAL_DONE ? "done, harmonics on" : st == COGCAL_FAILED_ABORTED ? "aborted" :
                                   st == COGCAL_FAILED_ERROR ? "failed: error or not calibrated" :
                                   st == COGCAL_FAILED_STUCK ? "failed: the shaft did not turn" :
                                   st == COGCAL_ENCODER ? "measuring the encoder table first" :
                                   st == COGCAL_FAILED_ENCODER ? "failed: the encoder table" :
                                   st == COGCAL_FAILED_TRIM ? "failed: the trim did not help" :
                                   st == COGCAL_STALE ? "off: the encoder table changed, identify again" :
                                   st == COGCAL_DONE_NO_TRIM ? "done, harmonics on, without the trim" : "idle");
                Serialport.print("Speed ");
                Serialport.print(cogcal_speed);
                Serialport.print(" ticks/s, about ");
                Serialport.print(2 * (16384UL / (cogcal_speed < 100 ? 100 : cogcal_speed) + 1));
                Serialport.print(" s; identified cogging ");
                Serialport.print(cogcal_ripple_x10 / 10.0f, 1);
                Serialport.println(" mA rms");
                if (st == COGCAL_DONE || st == COGCAL_DONE_NO_TRIM)
                    for (int s = 0; s < COGGING_HARMONICS; s++)
                    {
                        Serialport.print("  h");
                        Serialport.print(cogging.harmonic[s]);
                        Serialport.print(": ");
                        Serialport.print(cogging.a[s]);
                        Serialport.print(", ");
                        Serialport.println(cogging.b[s]);
                    }
            }

            // Product lock: #Product shows the board's stamp; #Product claim stamps it as this product (a motor
            // the rule misjudges, a repaired board); #Product clear removes the stamp. UART only, on purpose.
            else if (strcmp(command, "Product") == 0)
            {
                if (strcmp(argument, "claim") == 0)
                {
                    Product_set_stamp(PRODUCT_ID);
                    Serialport.println("Board stamped as this product. Power-cycle the drive.");
                }
                else if (strcmp(argument, "clear") == 0)
                {
                    Product_set_stamp(0);
                    Serialport.println("Stamp cleared. Power-cycle the drive.");
                }
                Serialport.print("Firmware: Spectral Micro; board stamp: ");
                Serialport.println(controller.board_product == 1 ? "STEPFOC" : controller.board_product == 2 ? "Spectral Micro" : "none");
                if (controller.wrong_product)
                    Serialport.println("WRONG FIRMWARE for this board: it drives nothing. Flash the right firmware.");
            }

            // Set/get the watchdog fallback: #WdFb time_ms,current_mA,speed_ticks_s,after_action
            // (after_action 0 legacy, 2 brake, 3 coast). Used by watchdog actions 1 (hold) and 4 (descend).
            // Saved with #Save.
            else if (strcmp(command, "WdFb") == 0)
            {
                if (strlen(argument) != 0)
                {
                    int w[4] = {0, 0, 0, 0};
                    const bool four = parse_ints(argument, w, 4) == 4;
                    int t = w[0], i = w[1], v = w[2], a = w[3];
                    if (four)
                    {
                        controller.wd_fallback_time_ms = t;
                        controller.wd_fallback_current = i;
                        controller.wd_fallback_speed = v;
                        controller.wd_after_action = a;
                        Settings2_sanitize();
                    }
                    else
                    {
                        Serialport.println("Use #WdFb time_ms,current_mA,speed_ticks_s,after_action");
                    }
                }
                Serialport.print("Watchdog fallback: ");
                Serialport.print(controller.wd_fallback_time_ms);
                Serialport.print(" ms, ");
                Serialport.print(controller.wd_fallback_current);
                Serialport.print(" mA, ");
                Serialport.print(controller.wd_fallback_speed);
                Serialport.print(" ticks/s, then ");
                Serialport.println(controller.wd_after_action);
            }

            // Calibrate gripper
            else if (strcmp(command, "Gripcal") == 0)
            {
                Gripper.cal_restart = 1; // from its first step, also after a run that was cut short
                PID.V_errSum = 0;
                PID.Id_errSum = 0;
                PID.Iq_errSum = 0;
                if (controller.reset_pin_state == 0 && controller.sleep_pin_state == 0)
                {
                    digitalWriteFast(SLEEP, HIGH);
                    digitalWriteFast(RESET, HIGH);
                    controller.reset_pin_state = 1;
                    controller.sleep_pin_state = 1;
                }
                controller.Controller_mode = 7;

                Serialport.println("Gripper calibration! ");
            }

            // Gripper info
            else if (strcmp(command, "Gripinfo") == 0)
            {

                if (controller.I_AM_GRIPPER == 0)
                {
                    Serialport.println("I am not a gripper ");
                }
                else
                {
                    Serialport.println("I am a gripper");
                }

                Serialport.print("Gripper calibrated is: ");
                Serialport.println(Gripper.calibrated);
                Serialport.print("Gripper activated is: ");
                Serialport.println(Gripper.activated);
                Serialport.print("Gripper position setpoint is: ");
                Serialport.println(Gripper.position_setpoint);
                Serialport.print("Gripper speed setpoint is: ");
                Serialport.println(Gripper.speed_setpoint);
                Serialport.print("Gripper current setpoint is: ");
                Serialport.println(Gripper.current_setpoint);
                Serialport.print("Gripper estop status: ");
                Serialport.println(Gripper.estop_status);
                Serialport.print("Gripper action status: ");
                Serialport.println(Gripper.action_status);
                Serialport.print("Gripper object detection status: ");
                Serialport.println(Gripper.object_detection_status);
                Serialport.print("Gripper max open position: ");
                Serialport.println(Gripper.max_open_position);
                Serialport.print("Gripper max closed position: ");
                Serialport.println(Gripper.max_closed_position);
            }

            // Set/Get gripper estop bit
            else if (strcmp(command, "Gripstop") == 0)
            {

                if (strlen(argument) != 0)
                {
                    Gripper.estop_status = atoi(argument);
                }
                Serialport.print("Gripper estop bit is: ");
                Serialport.println(Gripper.estop_status);
            }

            // Set/Get gripper activation bit
            else if (strcmp(command, "Gripact") == 0)
            {

                if (strlen(argument) != 0)
                {
                    Gripper.activated = atoi(argument);
                }
                Serialport.print("Gripper activation bit is: ");
                Serialport.println(Gripper.activated);
            }

            // Set/Get gripper current
            else if (strcmp(command, "Gripcur") == 0)
            {

                if (strlen(argument) != 0)
                {
                    Gripper.current_setpoint = atoi(argument);
                }
                Serialport.print("Gripper current setpoint is: ");
                Serialport.println(Gripper.current_setpoint);
            }

            // Set/Get gripper velocity
            else if (strcmp(command, "Gripvel") == 0)
            {

                if (strlen(argument) != 0)
                {
                    Gripper.speed_setpoint = atoi(argument);
                }
                Serialport.print("Gripper velocity setpoint is: ");
                Serialport.println(Gripper.speed_setpoint);
            }

            // Set/Get gripper position setpoint and go to gripper mode (goto action big)
            else if (strcmp(command, "Grippos") == 0)
            {

                if (strlen(argument) != 0)
                {
                    if (controller.reset_pin_state == 0 && controller.sleep_pin_state == 0)
                    {
                        digitalWriteFast(SLEEP, HIGH);
                        digitalWriteFast(RESET, HIGH);
                        controller.reset_pin_state = 1;
                        controller.sleep_pin_state = 1;
                    }

                    if (PID.Reset_integral_accumulator == 1)
                    {
                        PID.V_errSum = 0;
                        PID.Iq_errSum = 0;
                        PID.Id_errSum = 0;
                    }

                    Gripper.activated = 1;
                    Gripper.Same_command = true;
                    Gripper.At_position = 0;
                    Gripper.action_status = 1;
                    controller.Controller_mode = 6;
                    Gripper.position_setpoint = atoi(argument);
                }
                Serialport.print("Gripper current position is: ");
                Serialport.println(Gripper.position);
            }

            // Print all currently set PID parameters for every control mode
            else if (strcmp(command, "Param") == 0)
            {
                Serialport.print("Position loop Kp: ");
                Serialport.println(PID.Kp_p, 5);

                Serialport.print("Velocity loop Kp: ");
                Serialport.println(PID.Kp_v, 5);

                Serialport.print("Velocity loop Ki: ");
                Serialport.println(PID.Ki_v, 5);

                Serialport.print("Voltage limit: ");
                if (PID.Voltage_limit == 0)
                {
                    Serialport.println(controller.VBUS_mV);
                }
                else
                {
                    Serialport.println(PID.Voltage_limit);
                }

                Serialport.print("Velocity limit: ");
                Serialport.print(PID.Velocity_limit);
                Serialport.println(" TICKS/s");

                Serialport.print("Iq Current loop Kp: ");
                Serialport.println(PID.Kp_iq, 5);

                Serialport.print("Iq Current loop Ki: ");
                Serialport.println(PID.Ki_iq, 5);

                Serialport.print("Iq Current limit: ");
                Serialport.print(PID.Iq_current_limit);
                Serialport.println(" mA");

                Serialport.print("PD impedance controller KP ");
                Serialport.println(PID.KP, 5);

                Serialport.print("PD impedance controller KD ");
                Serialport.println(PID.KD, 5);
            }

            // Periodically send messages without request from host. Messeges are position, speed and Iq current.
            // Format of the command is #Cyc time. Time is variable in millisecond (needs to be integer)
            // If time is not given (or 0) command is disabled.
            // Format of cyclic output is: $Position Speed Torque\n
            // Cyclic can also be used with: https://github.com/nathandunk/BetterSerialPlotter
            else if (strcmp(command, "Cyc") == 0)
            {

                if (strlen(argument) > 0)
                {
                    controller.cyclic = 1;
                    controller.cyclic_a = 0;
                    controller.cyclic_period = atoi(argument);
                }
                else
                {
                    controller.cyclic = 0;
                    Serialport.println("Cyclic disabled");
                }
            }

            // Periodically send messages without request from host. Messeges are position, speed and Iq current.
            // Format of the command is #Cyca time. Time is variable in millisecond (needs to be integer)
            // If time is not given (or 0) command is disabled.
            // Format of cyclic output is: $,Position,Speed,Torque\n
            // Cyca can be used with arduino plotter
            else if (strcmp(command, "Cyca") == 0)
            {

                if (strlen(argument) > 0)
                {
                    controller.cyclic_a = 1;
                    controller.cyclic = 0;
                    controller.cyclic_period = atoi(argument);
                }
                else
                {
                    controller.cyclic_a = 0;
                    Serialport.println("Cyclic disabled");
                }
            }

            // Set/Get CAN ID
            else if (strcmp(command, "CANID") == 0)
            {

                if (strlen(argument) != 0)
                {
                    int temp_var = atoi(argument);
                    if (temp_var < 0 || temp_var > 15)
                    {
                        Serialport.println("Invalid CAN ID ");
                    }
                    else
                    {
                        controller.CAN_ID = temp_var;
                        if (controller.CAN_init_error == 0)
                            CANSetNodeFilter(controller.CAN_ID);
                    }
                }

                Serialport.print("CAN ID is: ");
                Serialport.println(controller.CAN_ID);
            }

            // Set/Get motor controller serial number
            else if (strcmp(command, "SERNUM") == 0)
            {

                if (strlen(argument) != 0)
                {
                    int temp_var = atoi(argument);
                    controller.SERIAL_NUMBER = temp_var;
                }

                Serialport.print("Serial number is: ");
                Serialport.println(controller.SERIAL_NUMBER);
            }

            // Set/Get motor calibration status
            else if (strcmp(command, "Calibrated") == 0)
            {
                if (strlen(argument) != 0)
                {
                    int var = atoi(argument);
                    if (abs(var) > 0)
                    {
                        controller.Calibrated = 1;
                    }
                    else
                    {
                        controller.Calibrated = 0;
                    }
                }
                if (controller.Calibrated == 0)
                {
                    Serialport.println("Not calibrated!");
                }
                else
                {
                    Serialport.println("Calibrated!");
                }
            }

            // Set/get Number of POLE PAIRS!
            else if (strcmp(command, "PP") == 0)
            {

                if (strlen(argument) != 0)
                {
                    // as parameter 78: 1-49, only while idle (any value used to be taken, in any mode)
                    if (!Param_set(PARAM_POLE_PAIRS, atoi(argument)))
                        Serialport.println("PP: not taken (1-49, and only while idle)");
                }
                Serialport.print("Pole pairs: ");
                Serialport.println(controller.pole_pairs);
            }

            // Activate the motor
            else if (strcmp(command, "Activate") == 0)
            {

                controller.Activated = 1;
                Serialport.println("Controller activated");
            }

            // Enter open loop mode and spin with speed; Example #Openloop 1000
            else if (strcmp(command, "Openloop") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.Open_loop_speed = parse_float(argument);
                }
                Serialport.print("Open loop speed: ");
                Serialport.println(controller.Open_loop_speed);
                if (controller.reset_pin_state == 0 && controller.sleep_pin_state == 0)
                {
                    digitalWriteFast(SLEEP, HIGH);
                    digitalWriteFast(RESET, HIGH);
                    controller.reset_pin_state = 1;
                    controller.sleep_pin_state = 1;
                }
                controller.Controller_mode = 5;
            }

            // Pull motor config
            else if (strcmp(command, "Pullconfig") == 0)
            {

                Serialport.print("%");
                Serialport.print(" ");
                Serialport.print(PID.Kp_p, 7);
                Serialport.print(" ");
                Serialport.print(PID.Kp_v, 7);
                Serialport.print(" ");
                Serialport.print(PID.Ki_v, 7);
                Serialport.print(" ");
                Serialport.print(PID.Velocity_limit);
                Serialport.print(" ");
                Serialport.print(PID.Kp_iq, 7);
                Serialport.print(" ");
                Serialport.print(PID.Ki_iq, 7);
                Serialport.print(" ");
                Serialport.print(PID.Iq_current_limit);
                Serialport.print(" ");
                Serialport.print(PID.KP, 7);
                Serialport.print(" ");
                Serialport.print(PID.KD, 7);
                Serialport.print(" ");

                Serialport.print(controller.Resistance, 7);
                Serialport.print(" ");
                Serialport.print(controller.Inductance, 7);
                Serialport.print(" ");
                Serialport.print(controller.pole_pairs);
                Serialport.print(" ");
                Serialport.print(controller.SERIAL_NUMBER);
                Serialport.print(" ");
                Serialport.print(controller.HARDWARE_VERSION);
                Serialport.print(" ");
                Serialport.print(controller.BATCH_DATE);
                Serialport.print(" ");
                Serialport.print(controller.SOFTWARE_VERSION);
                Serialport.print(" ");

                Serialport.print(controller.Calibrated);
                Serialport.print(" ");
                Serialport.print(controller.Error);
                Serialport.print(" ");
                Serialport.print(controller.temperature_error);
                Serialport.print(" ");
                Serialport.print(controller.encoder_error);
                Serialport.print(" ");
                Serialport.print(controller.Vbus_error);
                Serialport.print(" ");
                Serialport.print(controller.Driver_error);
                Serialport.print(" ");
                Serialport.print(controller.Velocity_error);
                Serialport.print(" ");
                Serialport.print(controller.Current_error);
                Serialport.print(" ");
                Serialport.print(controller.ESTOP_error);
                Serialport.print(" ");
                Serialport.print(controller.Watchdog_error);
                Serialport.print(" ");
                Serialport.print(controller.VBUS_mV);
                Serialport.println(" ");
            }

            // Enable disable LED status
            else if (strcmp(command, "LED") == 0)
            {

                if (strlen(argument) != 0)
                {
                    int var = atoi(argument);
                    if (abs(var) > 0)
                    {
                        controller.LED_ON_OFF = 1;
                    }
                    else
                    {
                        controller.LED_ON_OFF = 0;
                    }
                }
                if (controller.LED_ON_OFF == 0)
                {
                    Serialport.println("LED status is OFF");
                    digitalWriteFast(LED, LOW);
                }
                else
                {
                    Serialport.println("LED status is ON");
                }
            }

            // Set/Get do we want to reset integral accumulators after receiving new setopint
            else if (strcmp(command, "Rstint") == 0)
            {

                if (strlen(argument) != 0)
                {
                    PID.Reset_integral_accumulator = atoi(argument);
                }
                Serialport.print("Reseting integral accumulators: ");
                Serialport.println(PID.Reset_integral_accumulator);
            }

            else
            {
                Serialport.println("Unknown command");
            }

            parser.resetCommandAndArgument(command, argument);
            break;
        }
        else if (parser.lastCommandUnknown)
        {
            // A complete line was received but rejected by the parser itself (not in its
            // whitelist, or malformed) -- give feedback instead of silently doing nothing.
            parser.lastCommandUnknown = false;
            Serialport.println("Unknown command");
        }
    }
}

/// @brief Contains code for cyclic uart functions that send data without host request
/// @param Serialport
/// @param ms
void Cyclic_UART(Stream &Serialport, uint32_t ms)
{

    if (controller.cyclic == 1)
    {
        if (controller.cyclic_period > 0)
        {
            static uint32_t last_time_ = 0;
            if ((ms - last_time_) >= controller.cyclic_period) // run every x ms
            {
                Serialport.print("$");
                Serialport.print(" ");
                Serialport.print(controller.Position_Ticks);
                Serialport.print(" ");
                Serialport.print(controller.Velocity_Filter);
                Serialport.print(" ");
                Serialport.print(FOC.Iq);
                Serialport.println(" ");
                last_time_ = ms;
            }
        }
    }
    else if (controller.cyclic_a == 1)
    {

        /// Arduino serial plotter format
        if (controller.cyclic_period > 0)
        {
            static uint32_t last_time_ = 0;
            if ((ms - last_time_) >= controller.cyclic_period) // run every x ms
            {
                Serialport.print("Position:");
                Serialport.print(controller.Position_Ticks);
                Serialport.print(",");
                Serialport.print("Speed:");
                Serialport.print(controller.Velocity_Filter);
                Serialport.print(",");
                Serialport.print("Iq:");
                Serialport.println(FOC.Iq);
                last_time_ = ms;
            }
        }
    }
}

/// @brief One CAN parameter over UART (#Par, batch 20): "P<n> = <value>"; unknown numbers only if asked.
void Uart_param_print(Stream &Serialport, uint8_t n, bool say_unknown)
{
    int32_t v = 0;
    if (!Param_get(n, &v))
    {
        if (say_unknown)
        {
            Serialport.print("P");
            Serialport.print(n);
            Serialport.println(" unknown");
        }
        return;
    }
    Serialport.print("P");
    Serialport.print(n);
    Serialport.print(" = ");
    if (Param_is_float(n))
    {
        float f;
        memcpy(&f, &v, 4);
        Serialport.println(f, 6);
    }
    else
        Serialport.println(v);
}

/// @brief What is in force and what runs, in words, from the CAN parameters (#Info, batch 20).
void Uart_features_print(Stream &Serialport)
{
    int32_t f = 0, v = 0, w = 0;
    if (!Param_get(PARAM_FEATURES, &f))
        return;
    Serialport.println("In force:");
    Serialport.print("  encoder table: ");
    if (f & FEAT_ENCODER_IN_USE)
    {
        Param_get(PARAM_ENCAL_PP, &v);
        Serialport.print("on, error ");
        Serialport.print(v / 100.0f, 2);
        Serialport.println(" deg p-p");
    }
    else
        Serialport.println((f & FEAT_ENCODER_MEASURED) ? "measured, off" : "not measured");
    Serialport.print("  anti-cogging: ");
    if (f & FEAT_COGGING_ON)
        Serialport.print((f & FEAT_COGGING_TABLE) ? "on (table)" : "on (harmonics)");
    else
        Serialport.print((f & FEAT_COGGING_SET) ? "identified, off" : "not identified");
    Serialport.println((f & FEAT_COGGING_SAVED) ? ", saved" : "");
    Param_get(PARAM_ZERO_TRIM1, &v);
    Param_get(PARAM_ZERO_TRIM2, &w);
    Serialport.print("  current-sense zero trim: ");
    Serialport.print(v);
    Serialport.print(" / ");
    Serialport.print(w);
    Serialport.println((f & FEAT_ZERO_SWITCHING) ? " counts (switching zero in use)" : " counts");
    Param_get(PARAM_COMM_DELAY_US, &v);
    Serialport.print("  delay compensation: ");
    if (v)
    {
        Serialport.print(v);
        Serialport.println(" us");
    }
    else
        Serialport.println("off");
    Param_get(PARAM_SPEED_ESTIMATOR, &v);
    Param_get(PARAM_SPEED_PLL_BANDWIDTH, &w);
    Serialport.print("  speed estimate: ");
    if (v == 0)
        Serialport.println("moving average");
    else
    {
        Serialport.print(v == 1 ? "PLL, " : "direct PLL, ");
        Serialport.print(w);
        Serialport.println(" Hz");
    }
    Param_get(PARAM_INPUT_FILTER_HZ, &v);
    Param_get(PARAM_VEL_INT_DEADBAND, &w);
    Serialport.print("  input filter: ");
    if (v)
    {
        Serialport.print(v);
        Serialport.print(" Hz");
    }
    else
        Serialport.print("off");
    Serialport.print("; integral deadband: ");
    if (w)
    {
        Serialport.print(w);
        Serialport.println(" ticks");
    }
    else
        Serialport.println("off");
    if (f & (FEAT_RUN_CALIBRATION | FEAT_RUN_ENCODER | FEAT_RUN_COGGING | FEAT_RUN_PENDING))
    {
        Serialport.print("  running: ");
        Serialport.println((f & FEAT_RUN_CALIBRATION) ? "calibration" : (f & FEAT_RUN_ENCODER) ? "encoder table" :
                           (f & FEAT_RUN_COGGING) ? "anti-cogging identification" : "the runs after the calibration");
    }
}
