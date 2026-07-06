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
#include "bootloader_config.h"

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
                    PID.Iq_setpoint = atof(argument);
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
                    PID.Id_setpoint = atof(argument);
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
                    PID.Velocity_limit = atoi(argument);
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
                    PID.Kp_p = atof(argument);
                }
                Serialport.print("Kpp ");
                Serialport.println(PID.Kp_p, 5);
            }

            // Set/Get KP (PD loop)
            else if (strcmp(command, "KP") == 0)
            {
                if (strlen(argument) != 0)
                {
                    PID.KP = atof(argument);
                }
                Serialport.print("KP ");
                Serialport.println(PID.KP, 6);
            }

            // Set/Get KD (PD loop)
            else if (strcmp(command, "KD") == 0)
            {
                if (strlen(argument) != 0)
                {
                    PID.KD = atof(argument);
                }
                Serialport.print("KD ");
                Serialport.println(PID.KD, 6);
            }

            // Set/Get Kpv (Velocity loop Kp gain)
            else if (strcmp(command, "Kpv") == 0)
            {

                if (strlen(argument) != 0)
                {
                    PID.Kp_v = atof(argument);
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
                    PID.Kp_iq = atof(argument);
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
                    PID.Kp_id = atof(argument);
                }
                Serialport.print("Kpid ");
                Serialport.println(PID.Kp_id, 5);
            }

            // Set/Get Kpiv (Velocity loop integrator gain)
            else if (strcmp(command, "Kiv") == 0)
            {

                if (strlen(argument) != 0)
                {
                    PID.Ki_v = atof(argument);
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
                    PID.Ki_iq = atof(argument);
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
                    PID.Ki_id = atof(argument);
                }
                Serialport.print("Kiid ");
                Serialport.println(PID.Ki_id, 5);
            }

            // Set/Get Resistance
            else if (strcmp(command, "R") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.Resistance = atof(argument);
                }
                Serialport.print("R ");
                Serialport.println(controller.Resistance, 5);
            }

            // Set/Get Inductance
            else if (strcmp(command, "L") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.Inductance = atof(argument);
                }
                Serialport.print("L ");
                Serialport.println(controller.Inductance, 5);
            }

            // Set/Get Kt (Torque constant)
            else if (strcmp(command, "Kt") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.Kt = atof(argument);
                }
                Serialport.print("Kt ");
                Serialport.println(controller.Kt, 5);
            }

            // Set/Get KV (motor velocity constant)
            else if (strcmp(command, "KV") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.KV = atof(argument);
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
                    controller.flux_linkage = atof(argument);
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

            // Enter calibration mode
            else if (strcmp(command, "Cal") == 0)
            {
                if (controller.reset_pin_state == 0 && controller.sleep_pin_state == 0)
                {
                    digitalWriteFast(SLEEP, HIGH);
                    digitalWriteFast(RESET, HIGH);
                    controller.reset_pin_state = 1;
                    controller.sleep_pin_state = 1;
                }
                controller.Calibration = 1;
                Ticker_detach(TIM3);
                Ticker_init(TIM3, LOOP_FREQ, Update_IT_callback_calib);
            }

            // set get theta_offset
            else if (strcmp(command, "Offset") == 0)
            {

                if (strlen(argument) != 0)
                {
                    controller.theta_offset = atof(argument);
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
                        PID.Iq_current_limit = var;
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
                    Serialport.println("FAULT pin is HIGH");
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
                    controller.current_control_bandwidth = atof(argument);
                }
                Serialport.print("Current loop bandwidth: ");
                Serialport.println(controller.current_control_bandwidth, 5);
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
                    controller.Calibration_power = atof(argument);
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
                    controller.Open_loop_speed = atof(argument);
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
                Serialport.println("  Openloop <e> Open-loop speed mode (electrical speed)");
                Serialport.println("");
                Serialport.println("Calibration:");
                Serialport.println("  Cal          Run full calibration");
                Serialport.println("  R/L <ohm/H>  Set phase resistance/inductance before Cal");
                Serialport.println("  PP <n>       Set pole pairs before Cal");
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
                Serialport.print("ISR execution time: ");
                Serialport.print(controller.execution_time);
                Serialport.println(" us");
                Serialport.print("CAN ID is: ");
                Serialport.println(controller.CAN_ID);
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

            // Calibrate gripper
            else if (strcmp(command, "Gripcal") == 0)
            {
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
                    controller.pole_pairs = atoi(argument);
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
                    controller.Open_loop_speed = atof(argument);
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
