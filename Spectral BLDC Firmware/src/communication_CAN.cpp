/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    communication_CAN.cpp
 * @brief   This file provides code for CAN protocol
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

#include "communication_CAN.h"
#include "bootloader_config.h"

/*
The node with the lowest ID will always win the arbitration and therefore has the highest priority.
*/

/// Arbitration ID = 11 bits = 4 + 6 + 1; FIRST 4 MSB are ID, nexz 6 are command ID, and last bit is an error bit

CAN_msg_t CAN_TX_msg;
CAN_msg_t CAN_RX_msg;

#define DEBUG_COMS 0



/// @brief Extract Node Id, Node msg and Error bit from 11bit can ID frame
/// @param canId 11 bit can ID frame
void Extract_from_CAN_ID(unsigned int canId)
{
    // Extracting ID2 (first 4 MSB)
    unsigned int ID2 = (canId >> 7) & 0xF;

    // Extracting CAN Command (next 6 bits)
    unsigned int canCommand = (canId >> 1) & 0x3F;

    // Extracting Error Bit (last bit)
    unsigned int errorBit = canId & 0x1;
}

/// @brief Combine Node_ID, Command_ID and Error into standard 11 bit CAN ID
/// @param Node_ID 
/// @param Command_ID 
/// @param errorBit 
/// @return standard 11 bit CAN ID
unsigned int Combine_2_CAN_ID(unsigned int Node_ID, unsigned int Command_ID, bool errorBit)
{
    // Combine components into an 11-bit CAN ID
    unsigned int canId = 0;

    // Add ID2 (first 4 MSB)
    canId |= (Node_ID & 0xF) << 7;

    // Add CAN Command (next 6 bits)
    canId |= (Command_ID & 0x3F) << 1;

    // Add Error Bit (last bit)
    canId |= (errorBit & 0x1);

    return canId;
}


/// @brief  Setup CAN bus hardware
/// @details On failure, sets CAN_init_error and returns so UART stays usable
/// (a bricked board with no CAN transceiver connected could otherwise never be
/// reached over UART either).
void Setup_CAN_bus()
{
    bool ret = CANInit(CAN_1000KBPS, 2);
    if (!ret)
        controller.CAN_init_error = 1;
}


/// @brief CAN protocol
/// @param Serialport
/// @details Drains up to CAN_RX_BUDGET messages per call instead of just one (the 3-deep
/// hardware FIFO can silently drop frames within a single loop() pass otherwise) or an
/// unbounded while (which would let heavy bus traffic -- e.g. several other nodes'
/// heartbeats on a shared multi-drop CAN bus -- starve UART_protocol/CAN_watchdog/
/// CAN_heartbeat for as long as the bus stays busy).
void CAN_protocol(Stream &Serialport)
{
    const int CAN_RX_BUDGET = 8; // > 3-deep HW FIFO, so a normal burst drains in one call
    int processed = 0;

    while (CANMsgAvail() && processed < CAN_RX_BUDGET)
    {
        processed++;

        /// Get CAN msg from buffer
        CANReceive(&CAN_RX_msg);
        /// Unpack CAN ID 
        unsigned int Node_ID = (CAN_RX_msg.id >> 7) & 0xF;
        unsigned int Command_ID = (CAN_RX_msg.id >> 1) & 0x3F;
        unsigned int Error_bit = CAN_RX_msg.id & 0x1;
        

        /// Print CAN data
        #if (DEBUG_COMS > 0)
        Serialport.print("Node ID: ");
        Serialport.println(Node_ID);
        Serialport.print("Command ID: ");
        Serialport.println(Command_ID);
        Serialport.print("Error Bit: ");
        Serialport.println(Error_bit);
        Serialport.print("Msg length: ");
        Serialport.println(CAN_RX_msg.len);
        Serialport.println("");
        Serialport.print("Is standard frame: ");
        Serialport.println(CAN_RX_msg.type);
        Serialport.println("");
        #endif
  

        // If node ID matches the ID of THIS motor driver
        if (Node_ID == controller.CAN_ID)
        {

            // Perform action depending on command ID we received
            switch (Command_ID)
            {
            case IN_DATA_PACK_1:{

                // If len = 8 positon loop, if len = 5 velocity loop, if len = 2 current loop

                // Positon loop
                if(CAN_RX_msg.len == 8){
                    uint8_t temp_buffer[] =  {CAN_RX_msg.data[0], CAN_RX_msg.data[1], CAN_RX_msg.data[2]};
                    uint8_t temp_buffer1[] =  {CAN_RX_msg.data[3], CAN_RX_msg.data[4], CAN_RX_msg.data[5]};
                    uint8_t temp_buffer2[] =  {CAN_RX_msg.data[6], CAN_RX_msg.data[7]};
                    PID.Position_setpoint = three_bytes_to_int(temp_buffer);
                    PID.Feedforward_speed = three_bytes_to_int(temp_buffer1);
                    PID.Feedforward_current = two_bytes_to_int(temp_buffer2);

                    if (PID.Reset_integral_accumulator == 1)
                    {
                        PID.V_errSum = 0;
                        PID.Iq_errSum = 0;
                        PID.Id_errSum = 0;
                    }

                    if(controller.Error  == 0){
                        if(controller.reset_pin_state == 0 && controller.sleep_pin_state == 0){
                            digitalWriteFast(SLEEP, HIGH);
                            digitalWriteFast(RESET, HIGH);
                            controller.reset_pin_state = 1;
                            controller.sleep_pin_state = 1;
                        }
                    }
                    controller.Controller_mode = 1;
                    controller.Wrong_DL = 0;
                    controller.watchdog_reset = 1;

                    #if (DEBUG_COMS > 0)
                    Serialport.print("Commanded positon: ");
                    Serialport.println(PID.Position_setpoint);
                    Serialport.print("Feedforward speed: ");
                    Serialport.println(PID.Feedforward_speed );
                    Serialport.print("Feedforward current: ");
                    Serialport.println(PID.Feedforward_current);
                    #endif

                // Speed loop
                }else if(CAN_RX_msg.len == 5){
                    uint8_t temp_buffer1[] =  {CAN_RX_msg.data[0], CAN_RX_msg.data[1], CAN_RX_msg.data[2]};
                    uint8_t temp_buffer2[] =  {CAN_RX_msg.data[3], CAN_RX_msg.data[4]};
                    PID.Velocity_setpoint = three_bytes_to_int(temp_buffer1);
                    PID.Feedforward_current = two_bytes_to_int(temp_buffer2);


                    if (PID.Reset_integral_accumulator == 1)
                    {
                        PID.V_errSum = 0;
                        PID.Iq_errSum = 0;
                        PID.Id_errSum = 0;
                    }
                    

                    if(controller.Error  == 0){
                        if(controller.reset_pin_state == 0 && controller.sleep_pin_state == 0){
                            digitalWriteFast(SLEEP, HIGH);
                            digitalWriteFast(RESET, HIGH);
                            controller.reset_pin_state = 1;
                            controller.sleep_pin_state = 1;
                        }
                    }
                    controller.Controller_mode = 2;
                    controller.Wrong_DL = 0;
                    controller.watchdog_reset = 1;

                    #if (DEBUG_COMS > 0)
                    Serialport.print("Commanded velocity: ");
                    Serialport.println(PID.Position_setpoint);
                    Serialport.print("Feedforward current: ");
                    Serialport.println(PID.Feedforward_current);
                    #endif

                // Current loop
                }else if(CAN_RX_msg.len == 2){
                    uint8_t temp_buffer2[] =  {CAN_RX_msg.data[0], CAN_RX_msg.data[1]};
                    PID.Iq_setpoint = two_bytes_to_int(temp_buffer2);

                    if (PID.Reset_integral_accumulator == 1)
                    {
                        PID.V_errSum = 0;
                        PID.Iq_errSum = 0;
                        PID.Id_errSum = 0;
                    }

                    if(controller.Error  == 0){
                        if(controller.reset_pin_state == 0 && controller.sleep_pin_state == 0){
                            digitalWriteFast(SLEEP, HIGH);
                            digitalWriteFast(RESET, HIGH);
                            controller.reset_pin_state = 1;
                            controller.sleep_pin_state = 1;
                        }
                    }
                    controller.Controller_mode = 3;
                    controller.Wrong_DL = 0;
                    controller.watchdog_reset = 1;

                    #if (DEBUG_COMS > 0)
                    Serialport.print("Commanded current: ");
                    Serialport.println(PID.Iq_setpoint);
                    #endif
                
                // If we received correct ID but wrong data format (length)
                }else{
                    controller.Wrong_DL = 1;
                    #if (DEBUG_COMS > 0)
                    Serialport.println("DATA_PACK_1; wrong DL");
                    #endif
                }
                
                // Always respond with this
                Data_pack_1_CAN();
                break;
            }

            case IN_DATA_PACK_2:{
                break;
            }

            case IN_DATA_PACK_3:{
                break;
            }

            case IN_DATA_PACK_PD:{

                
                if(CAN_RX_msg.len == 8){
                    uint8_t temp_buffer[] =  {CAN_RX_msg.data[0], CAN_RX_msg.data[1], CAN_RX_msg.data[2]};
                    uint8_t temp_buffer1[] =  {CAN_RX_msg.data[3], CAN_RX_msg.data[4], CAN_RX_msg.data[5]};
                    uint8_t temp_buffer2[] =  {CAN_RX_msg.data[6], CAN_RX_msg.data[7]};
                    PID.Position_setpoint = three_bytes_to_int(temp_buffer);
                    PID.Feedforward_speed = three_bytes_to_int(temp_buffer1);
                    PID.Feedforward_current = two_bytes_to_int(temp_buffer2);

                    if (PID.Reset_integral_accumulator == 1)
                    {
                        PID.V_errSum = 0;
                        PID.Iq_errSum = 0;
                        PID.Id_errSum = 0;
                    }

                    if(controller.Error  == 0){
                        if(controller.reset_pin_state == 0 && controller.sleep_pin_state == 0){
                            digitalWriteFast(SLEEP, HIGH);
                            digitalWriteFast(RESET, HIGH);
                            controller.reset_pin_state = 1;
                            controller.sleep_pin_state = 1;
                        }
                    }
                    controller.Controller_mode = 4;
                    controller.Wrong_DL = 0;
                    controller.watchdog_reset = 1;

                    #if (DEBUG_COMS > 0)
                    Serialport.print("PD Commanded positon: ");
                    Serialport.println(PID.Position_setpoint);
                    Serialport.print("PD Commanded speed: ");
                    Serialport.println(PID.Velocity_setpoint );
                    Serialport.print("PD Commanded current: ");
                    Serialport.println(PID.Feedforward_current);
                    #endif
                }else{
                    controller.Wrong_DL = 1;
                    #if (DEBUG_COMS > 0)
                    Serialport.println("DATA_PACK_PD; wrong DL");
                    #endif
                }
                
                Data_pack_1_CAN();
                break;
 
            }

            case IN_GRIPPER_DATA_PACK:{

                if(CAN_RX_msg.len == 5){

                    uint8_t currentBytes[5];
                    for (int i = 0; i < 5; i++) {
                        currentBytes[i] = CAN_RX_msg.data[i];
                        }
                    // If current command is same as prev command
                    Gripper.Same_command = true;
                    for (int i = 0; i < 5; i++) {
                        if (currentBytes[i] != Gripper.previousBytes[i]) {
                            Gripper.Same_command = false;
                            break;
                        }
                        }

                    if (PID.Reset_integral_accumulator == 1)
                    {
                        PID.V_errSum = 0;
                        PID.Iq_errSum = 0;
                        PID.Id_errSum = 0;
                    }

                    Gripper.position_setpoint = CAN_RX_msg.data[0]; // position setpoint
                    Gripper.speed_setpoint = CAN_RX_msg.data[1]; // speed setpoint
                    uint8_t temp_buffer[] =  {CAN_RX_msg.data[2], CAN_RX_msg.data[3]};
                    Gripper.current_setpoint = two_bytes_to_int(temp_buffer); // current setpoint
                    bool bitArray[8];
                    byteToBitsBigEndian(CAN_RX_msg.data[4],bitArray);
                    Gripper.activated = bitArray[0];
                    Gripper.action_status = bitArray[1];
                    Gripper.estop_status = bitArray[2];
                    Gripper.release_direction = bitArray[3];
                    
                    if(controller.Error  == 0){
                        if(controller.reset_pin_state == 0 && controller.sleep_pin_state == 0){
                            digitalWriteFast(SLEEP, HIGH);
                            digitalWriteFast(RESET, HIGH);
                            controller.reset_pin_state = 1;
                            controller.sleep_pin_state = 1;
                        }
                    }

                    controller.Controller_mode = 6;
                    controller.watchdog_reset = 1;
                    controller.Wrong_DL = 0;

                    // Update previous bytes
                    for (int i = 0; i < 5; i++) {
                        Gripper.previousBytes[i] = currentBytes[i];
                        }
                    
                    #if (DEBUG_COMS > 0)
                    Serialport.println("Received gripper data");
                    Serialport.print("Commanded positon: ");
                    Serialport.println(Gripper.position_setpoint);
                    Serialport.print("Commanded speed: ");
                    Serialport.println(Gripper.speed_setpoint);
                    Serialport.print("Commanded current: ");
                    Serialport.println(Gripper.current_setpoint);
                    Serialport.print("Gripper activation command: ");
                    Serialport.println(Gripper.activated);
                    Serialport.print("Gripper action command: ");
                    Serialport.println(Gripper.action_status);
                    Serialport.print("Gripper estop command: ");
                    Serialport.println(Gripper.estop_status);
                    Serialport.print("Gripper release direction command: ");
                    Serialport.println(Gripper.release_direction);
                    Serialport.print("Gripper is same command: ");
                    Serialport.println(Gripper.Same_command);
                    #endif
                
                }else if (CAN_RX_msg.len == 0){
                    #if (DEBUG_COMS > 0)
                    Serialport.println("Gripper empty command ");
                    #endif
                    Gripper.Same_command = true;
                    //controller.Controller_mode = 6;
                    controller.watchdog_reset = 1;
                    controller.Wrong_DL = 0;

                // If we received correct ID but wrong data format 
                }else{
                    controller.Wrong_DL = 1;
                    #if (DEBUG_COMS > 0)
                    Serialport.println("GRIPPER DATA; Wrong DL");
                    #endif
                }

                Gripper_pack_data();
                break;
            }

            case IN_WATCHDOG_TIMEOUT:{
                if(CAN_RX_msg.len == 5){
                    uint8_t temp_buffer[] =  {CAN_RX_msg.data[0], CAN_RX_msg.data[1], CAN_RX_msg.data[2], CAN_RX_msg.data[3]};
                    controller.watchdog_time_ms = fourBytesToInt(temp_buffer);
                    controller.watchdog_action = CAN_RX_msg.data[4];
                    controller.Wrong_DL = 0;
                    #if (DEBUG_COMS > 0)
                    Serialport.print("New watchdog timeout time is: ");
                    Serialport.println(controller.watchdog_time_ms);
                    Serialport.print("Action is: ");
                    Serialport.println(controller.watchdog_action);
                    #endif
                }else{
                    controller.Wrong_DL = 1;
                    #if (DEBUG_COMS > 0)
                    Serialport.println("Watchdog; Wrong DL");
                    #endif  
                }

                break;
            }

            case HEARTBEAT_SETUP:{
                if(CAN_RX_msg.len == 4){
                    uint8_t temp_buffer[] =  {CAN_RX_msg.data[0], CAN_RX_msg.data[1], CAN_RX_msg.data[2], CAN_RX_msg.data[3]};
                    controller.Heartbeat_rate_ms = fourBytesToInt(temp_buffer);
                    controller.Wrong_DL = 0;
                    #if (DEBUG_COMS > 0)
                    Serialport.print("New heartbeat rate is: ");
                    Serialport.println(controller.Heartbeat_rate_ms);
                    #endif
                }else{
                    controller.Wrong_DL = 1;
                    #if (DEBUG_COMS > 0)
                    Serialport.println("Heartbeat; Wrong DL");
                    #endif  
                }

                break;
            }

            case IN_ESTOP:{
                controller.ESTOP_error = 1;
                #if (DEBUG_COMS > 0)
                Serialport.println("Received ESTOP");
                #endif
                break;
            }

            case IN_IDLE:{
                controller.Controller_mode = 0;
                #if (DEBUG_COMS > 0)
                Serialport.println("Received Idle");
                #endif
                break;
            }

            case IN_SAVE_CONFIG:{
                Write_config();
                #if (DEBUG_COMS > 0)
                Serialport.println("Received Save config");
                #endif
                break;
            }

            case IN_RESET:{
                #if (DEBUG_COMS > 0)
                Serialport.println("Received Reset");
                #endif
                Bootloader_ResetIntoBootloader();
                break;
            }

            case IN_CLEAR_ERROR:{
                #if (DEBUG_COMS > 0)
                Serialport.println("Received Clear Error");
                #endif
                if(controller.reset_pin_state == 0 && controller.sleep_pin_state == 0){
                    digitalWriteFast(SLEEP, HIGH);
                    digitalWriteFast(RESET, HIGH);
                    controller.reset_pin_state = 1;
                    controller.sleep_pin_state = 1;
                }
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
                break;
            }
                
            case IN_KP_KD:{
                if(CAN_RX_msg.len == 8){
                    uint8_t temp_buffer[] =  {CAN_RX_msg.data[0], CAN_RX_msg.data[1], CAN_RX_msg.data[2], CAN_RX_msg.data[3]};
                    uint8_t temp_buffer2[] =  {CAN_RX_msg.data[4], CAN_RX_msg.data[5], CAN_RX_msg.data[6], CAN_RX_msg.data[7]};
                    PID.KP = fourBytesToFloat(temp_buffer);
                    PID.KD = fourBytesToFloat(temp_buffer2);
                    controller.Wrong_DL = 0;
                    #if (DEBUG_COMS > 0)
                    Serialport.print("New PD loop KP gain: ");
                    Serialport.println(PID.KP,5);
                    Serialport.print("New PD loop KD gain: ");
                    Serialport.println(PID.KD,5);
                    #endif
                }else{
                    #if (DEBUG_COMS > 0)
                    Serialport.println("KP_KD; Wrong DL");
                    #endif 
                    controller.Wrong_DL = 1;
                }
                break;
            }

            case IN_KIIQ_KPIQ:{
                if(CAN_RX_msg.len == 8){
                    uint8_t temp_buffer[] =  {CAN_RX_msg.data[0], CAN_RX_msg.data[1], CAN_RX_msg.data[2], CAN_RX_msg.data[3]};
                    uint8_t temp_buffer2[] =  {CAN_RX_msg.data[4], CAN_RX_msg.data[5], CAN_RX_msg.data[6], CAN_RX_msg.data[7]};
                    PID.Kp_iq = fourBytesToFloat(temp_buffer);
                    PID.Ki_iq = fourBytesToFloat(temp_buffer2);
                    controller.Wrong_DL = 0;
                    #if (DEBUG_COMS > 0)
                    Serialport.print("New Iq current loop Kp gain: ");
                    Serialport.println(PID.Kp_iq,5);
                    Serialport.print("New Iq loop integrator gain: ");
                    Serialport.println(PID.Ki_iq,5);
                    #endif
                }else{
                    #if (DEBUG_COMS > 0)
                    Serialport.println("KIIQ_KPIQ; Wrong DL");
                    #endif 
                    controller.Wrong_DL = 1;
                }
                break;
            }

            case IN_KPV_KIV:{
                if(CAN_RX_msg.len == 8){
                    uint8_t temp_buffer[] =  {CAN_RX_msg.data[0], CAN_RX_msg.data[1], CAN_RX_msg.data[2], CAN_RX_msg.data[3]};
                    uint8_t temp_buffer2[] =  {CAN_RX_msg.data[4], CAN_RX_msg.data[5], CAN_RX_msg.data[6], CAN_RX_msg.data[7]};
                    PID.Kp_v = fourBytesToFloat(temp_buffer);
                    PID.Ki_v = fourBytesToFloat(temp_buffer2);
                    controller.Wrong_DL = 0;
                    #if (DEBUG_COMS > 0)
                    Serialport.print("New Velocity loop Kp gain: ");
                    Serialport.println(PID.Kp_v,5);
                    Serialport.print("New Velocity loop integrator gain: ");
                    Serialport.println(PID.Ki_v,5);
                    #endif
                }else{
                    #if (DEBUG_COMS > 0)
                    Serialport.println("KPV_KIV; Wrong DL");
                    #endif 
                    controller.Wrong_DL = 1;
                }
                break;
            }

            case IN_KPP:{
                if(CAN_RX_msg.len == 4){
                    uint8_t temp_buffer[] =  {CAN_RX_msg.data[0], CAN_RX_msg.data[1], CAN_RX_msg.data[2], CAN_RX_msg.data[3]};
                    PID.Kp_p = fourBytesToFloat(temp_buffer);
                    controller.Wrong_DL = 0;
                    #if (DEBUG_COMS > 0)
                    Serialport.print("New Position loop Kp gain: ");
                    Serialport.println(PID.Kp_p,5);
                    #endif
                }else{
                    #if (DEBUG_COMS > 0)
                    Serialport.println("KPP; Wrong DL");
                    #endif 
                    controller.Wrong_DL = 1;   
                }
                break;
            }

            case IN_LIMITS:{
                if(CAN_RX_msg.len == 8){
                    uint8_t temp_buffer[] =  {CAN_RX_msg.data[0], CAN_RX_msg.data[1], CAN_RX_msg.data[2], CAN_RX_msg.data[3]};
                    uint8_t temp_buffer2[] =  {CAN_RX_msg.data[4], CAN_RX_msg.data[5], CAN_RX_msg.data[6], CAN_RX_msg.data[7]};
                    PID.Velocity_limit = fourBytesToFloat(temp_buffer);
                    PID.Iq_current_limit = fourBytesToFloat(temp_buffer2);
                    controller.Wrong_DL = 0;
                    #if (DEBUG_COMS > 0)
                    Serialport.print("New Speed limit is: ");
                    Serialport.println(PID.Velocity_limit,5);
                    Serialport.print("New Current limit is: ");
                    Serialport.println(PID.Iq_current_limit,5);
                    #endif
                }else{
                    #if (DEBUG_COMS > 0)
                    Serialport.println("LIMITS; Wrong DL");
                    #endif 
                    controller.Wrong_DL = 1;   
                }

                break;
            }

            case IN_VLIM:{
                if(CAN_RX_msg.len == 4){
                    uint8_t temp_buffer[] =  {CAN_RX_msg.data[0], CAN_RX_msg.data[1], CAN_RX_msg.data[2], CAN_RX_msg.data[3]};
                    PID.Voltage_limit = fourBytesToInt(temp_buffer);
                    controller.Wrong_DL = 0;
                    #if (DEBUG_COMS > 0)
                    Serialport.print("New Voltage limit is: ");
                    Serialport.println(PID.Voltage_limit);
                    #endif
                }else{
                    #if (DEBUG_COMS > 0)
                    Serialport.println("VLIM; Wrong DL");
                    #endif
                    controller.Wrong_DL = 1;
                }

                break;
            }

            case IN_CAN_ID:{
                if(CAN_RX_msg.len == 1){
                    uint8_t new_can_id = CAN_RX_msg.data[0];
                    if (spectral_can_id_is_valid(new_can_id)) {
                        controller.CAN_ID = new_can_id;
                        controller.Wrong_DL = 0;
                        #if (DEBUG_COMS > 0)
                        Serialport.print("New CAN ID is: ");
                        Serialport.println(new_can_id);
                        #endif
                    } else {
                        controller.Wrong_DL = 1;
                        #if (DEBUG_COMS > 0)
                        Serialport.println("CANID; Invalid ID");
                        #endif
                    }
                }else{
                    #if (DEBUG_COMS > 0)
                    Serialport.println("CANID; Wrong DL");
                    #endif 
                    controller.Wrong_DL = 1;   
                }
                break;
            }

            case IN_KT:{
                if(CAN_RX_msg.len == 4){
                    uint8_t temp_buffer[] =  {CAN_RX_msg.data[0], CAN_RX_msg.data[1], CAN_RX_msg.data[2], CAN_RX_msg.data[3]};
                    controller.Kt = fourBytesToFloat(temp_buffer);
                    controller.Wrong_DL = 0;
                    #if (DEBUG_COMS > 0)
                    Serialport.print("New Kt is: ");
                    Serialport.println(controller.Kt,5);
                    #endif
                }else{
                     #if (DEBUG_COMS > 0)
                    Serialport.println("KT; Wrong DL");
                    #endif 
                    controller.Wrong_DL = 1;     
                }
                
                break;
            }

            case IN_GRIPPER_CALIB:{
                
                PID.V_errSum = 0;
                PID.Id_errSum = 0;
                PID.Iq_errSum = 0;
                if(controller.reset_pin_state == 0 && controller.sleep_pin_state == 0){
                    digitalWriteFast(SLEEP, HIGH);
                    digitalWriteFast(RESET, HIGH);
                    controller.reset_pin_state = 1;
                    controller.sleep_pin_state = 1;
                }
                controller.Controller_mode = 7;

                #if (DEBUG_COMS > 0)
                Serialport.print("Received calibrate gripper ");
                #endif        
                break;
            }


            case OUT_IN_PING:{
                #if (DEBUG_COMS > 0)
                Serialport.println("PING request");
                #endif
                if (CAN_RX_msg.type == REMOTE_FRAME)
                {
                    Ping_response();
                    controller.watchdog_reset = 1;
                }
                break;
            }

            case OUT_IN_ENCODER:{
                #if (DEBUG_COMS > 0)
                Serialport.println("Encoder data request");
                #endif
                if (CAN_RX_msg.type == REMOTE_FRAME)
                {
                    Encoder_data_CAN();
                    controller.watchdog_reset = 1;
                }
                break;
            }

            case OUT_IN_KT:{
                #if (DEBUG_COMS > 0)
                Serialport.println("Kt request");
                #endif
                if (CAN_RX_msg.type == REMOTE_FRAME)
                {
                    KT_data_CAN();
                    controller.watchdog_reset = 1;
                }
                break;
            }

            case OUT_IN_IQ:{
                #if (DEBUG_COMS > 0)
                Serialport.println("Current/Torque request");
                #endif
                if (CAN_RX_msg.type == REMOTE_FRAME)
                {
                    Current_data_CAN();
                    controller.watchdog_reset = 1;
                }
                break;
            }

            case OUT_IN_STATE_OF_ERRORS_CAN:{
                #if (DEBUG_COMS > 0)
                Serialport.println("State of errors request");
                #endif
                if (CAN_RX_msg.type == REMOTE_FRAME)
                {
                    State_of_Errors_CAN();
                    controller.watchdog_reset = 1;
                }
                break;
            }

            case OUT_IN_DEVICE_INFO_CAN:{
                #if (DEBUG_COMS > 0)
                Serialport.println("Device info request");
                #endif
                if (CAN_RX_msg.type == REMOTE_FRAME)
                {
                    Device_info_CAN();
                    controller.watchdog_reset = 1;
                }
                break;
            }

            case OUT_IN_VOLTAGE_CAN:{
                #if (DEBUG_COMS > 0)
                Serialport.println("Voltage request");
                #endif
                if (CAN_RX_msg.type == REMOTE_FRAME)
                {
                    Voltage_CAN();
                    controller.watchdog_reset = 1;
                }
                break;
            }

            case OUT_IN_TEMPERATURE_CAN:{
                #if (DEBUG_COMS > 0)
                Serialport.println("Temperature request");
                #endif
                if (CAN_RX_msg.type == REMOTE_FRAME)
                {
                    Temperature_CAN();
                    controller.watchdog_reset = 1;
                }
                break;
            }

            }
        } else{
                #if (DEBUG_COMS > 0)
                Serialport.println("Wrong Node ID ");
                #endif

        }
    }
}


/// @todo This function will allow motor controller to send CAN commands with desired interval without host request
/// @brief 
/// @param ms 
void Cyclic_CAN(uint32_t ms)
{

    if (1)
    {
        static uint32_t last_time_ = 0;
        if ((ms - last_time_) >= 100) // run every x ms
        {
            last_time_ = ms;
        }
    }
}


/// @brief Send heartbeat data ever ms interval
/// @param ms 
void CAN_heartbeat(uint32_t ms)
{

    if (controller.Heartbeat_rate_ms != 0)
    {
        static uint32_t last_time_ = 0;
        /// If we sent any other msg reset send heartbeat
        if(controller.Send_heartbeat == 0){
            last_time_ = ms;
            controller.Send_heartbeat = 1;
        }
        /// If we did not send any other msg send heartbeat
        if ((ms - last_time_) >= controller.Heartbeat_rate_ms) // run every x ms
        {
            Heartbeat_CAN();
            last_time_ = ms;
            controller.Send_heartbeat = 1;
        }
    }
}


/// @brief If we dont get command after ms time get watchdog error. Receiving any valid command resets this timer.
/// @param ms 
void CAN_watchdog(uint32_t ms){
    static uint32_t last_time_ = 0;
    if(controller.watchdog_time_ms != 0){

        if (controller.watchdog_reset == 1){
                last_time_ = ms;
                controller.watchdog_reset = 0;
        }

        if ((ms - last_time_) >= controller.watchdog_time_ms) // run every x ms
        {
         controller.Watchdog_error = 1;
         last_time_ = ms;
        }
    }
}


/// @brief Heartbeat CAN msg (no data)
/// Direction Driver -> host
void Heartbeat_CAN()
{
    CAN_TX_msg.data[0] = 0x00; 
    CAN_TX_msg.data[1] = 0x00;
    CAN_TX_msg.data[2] = 0x00;
    CAN_TX_msg.data[3] = 0x00;
    CAN_TX_msg.data[4] = 0x00;
    CAN_TX_msg.data[5] = 0x00;
    CAN_TX_msg.data[6] = 0x00;
    CAN_TX_msg.data[7] = 0x00;
    CAN_TX_msg.len = 0;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_HEARTBEAT, controller.Error);
    CANSend(&CAN_TX_msg);
}

/// @brief  Ping (no data)
/// Direction Driver -> host
void Ping_response()
{
    CAN_TX_msg.data[0] = 0x00; 
    CAN_TX_msg.data[1] = 0x00;
    CAN_TX_msg.data[2] = 0x00;
    CAN_TX_msg.data[3] = 0x00;
    CAN_TX_msg.data[4] = 0x00;
    CAN_TX_msg.data[5] = 0x00;
    CAN_TX_msg.data[6] = 0x00;
    CAN_TX_msg.data[7] = 0x00;
    CAN_TX_msg.len = 0;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_IN_PING, controller.Error);
    CANSend(&CAN_TX_msg);
    controller.Send_heartbeat = 0;
}

/// @brief Send motor temperature (2 byte)
/// Direction Driver -> host
void Temperature_CAN()
{
    byte data_buffer_send[2];
    intTo2Bytes(controller.TEMP_DEG, data_buffer_send);
    CAN_TX_msg.data[0] = data_buffer_send[0];
    CAN_TX_msg.data[1] = data_buffer_send[1];
    CAN_TX_msg.data[2] = 0x00;
    CAN_TX_msg.data[3] = 0x00;
    CAN_TX_msg.data[4] = 0x00;
    CAN_TX_msg.data[5] = 0x00;
    CAN_TX_msg.data[6] = 0x00;
    CAN_TX_msg.data[7] = 0x00;
    CAN_TX_msg.len = 2;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_IN_TEMPERATURE_CAN, controller.Error);
    CANSend(&CAN_TX_msg);
    controller.Send_heartbeat = 0;
}

/// @brief Send motor Vbus voltage (2 byte)
/// Direction Driver -> host
void Voltage_CAN()
{
    byte data_buffer_send[2];
    intTo2Bytes(controller.VBUS_mV, data_buffer_send);
    CAN_TX_msg.data[0] = data_buffer_send[0];
    CAN_TX_msg.data[1] = data_buffer_send[1];
    CAN_TX_msg.data[2] = 0x00;
    CAN_TX_msg.data[3] = 0x00;
    CAN_TX_msg.data[4] = 0x00;
    CAN_TX_msg.data[5] = 0x00;
    CAN_TX_msg.data[6] = 0x00;
    CAN_TX_msg.data[7] = 0x00;
    CAN_TX_msg.len = 2;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_IN_VOLTAGE_CAN, controller.Error);
    CANSend(&CAN_TX_msg);
    controller.Send_heartbeat = 0;
}


/// @brief Send motor info ( 7 byte)
/// Direction Driver -> host
void Device_info_CAN()
{
    byte data_buffer_send[4];
    intTo4Bytes(controller.SERIAL_NUMBER, data_buffer_send);
    CAN_TX_msg.data[0] = controller.HARDWARE_VERSION;
    CAN_TX_msg.data[1] = controller.BATCH_DATE;
    CAN_TX_msg.data[2] = controller.SOFTWARE_VERSION;
    CAN_TX_msg.data[3] = data_buffer_send[0];
    CAN_TX_msg.data[4] = data_buffer_send[1];
    CAN_TX_msg.data[5] = data_buffer_send[2];
    CAN_TX_msg.data[6] = data_buffer_send[3];
    CAN_TX_msg.data[7] = 0x00;
    CAN_TX_msg.len = 7;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_IN_DEVICE_INFO_CAN, controller.Error);
    CANSend(&CAN_TX_msg);
    controller.Send_heartbeat = 0;
}

/// @brief Send motor position and speed (8 byte)
/// Direction Driver -> host
void Encoder_data_CAN()
{
    byte data_buffer_send[4];
    intTo4Bytes(controller.Position_Ticks, data_buffer_send);
    CAN_TX_msg.data[0] = data_buffer_send[0];
    CAN_TX_msg.data[1] = data_buffer_send[1];
    CAN_TX_msg.data[2] = data_buffer_send[2];
    CAN_TX_msg.data[3] = data_buffer_send[3];
    intTo4Bytes(controller.Velocity_Filter, data_buffer_send);
    CAN_TX_msg.data[4] = data_buffer_send[0];
    CAN_TX_msg.data[5] = data_buffer_send[1];
    CAN_TX_msg.data[6] = data_buffer_send[2];
    CAN_TX_msg.data[7] = data_buffer_send[3];
    CAN_TX_msg.len = 8;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_IN_ENCODER, controller.Error);
    CANSend(&CAN_TX_msg);
    controller.Send_heartbeat = 0;
}

/// @brief Send motor current data (2 byte)
/// Direction Driver -> host
void Current_data_CAN()
{
    byte data_buffer_send[2];
    intTo2Bytes((int)FOC.Iq, data_buffer_send);
    CAN_TX_msg.data[0] = data_buffer_send[0];
    CAN_TX_msg.data[1] = data_buffer_send[1];
    CAN_TX_msg.data[2] = 0x00;
    CAN_TX_msg.data[3] = 0x00;
    CAN_TX_msg.data[4] = 0x00;
    CAN_TX_msg.data[5] = 0x00;
    CAN_TX_msg.data[6] = 0x00;
    CAN_TX_msg.data[7] = 0x00;
    CAN_TX_msg.len = 2;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_IN_IQ, controller.Error);
    CANSend(&CAN_TX_msg);
    controller.Send_heartbeat = 0;
}


/// @brief Send Kt (motor torque constant) value
/// Direction Driver -> host (response to REMOTE_FRAME on OUT_IN_KT)
void KT_data_CAN()
{
    // Pack float as 4 bytes big-endian (same byte order as fourBytesToFloat)
    union { float f; uint32_t i; } data;
    data.f = controller.Kt;
    CAN_TX_msg.data[0] = (data.i >> 24) & 0xFF;
    CAN_TX_msg.data[1] = (data.i >> 16) & 0xFF;
    CAN_TX_msg.data[2] = (data.i >> 8)  & 0xFF;
    CAN_TX_msg.data[3] =  data.i        & 0xFF;
    CAN_TX_msg.len = 4;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_IN_KT, controller.Error);
    CANSend(&CAN_TX_msg);
    controller.Send_heartbeat = 0;
}


/// @brief  Send state of all errors in motor driver (2 byte)
/// Direction Driver -> host
void State_of_Errors_CAN()
{
    bool Error_array[] = {controller.Error, controller.temperature_error, controller.encoder_error,
                          controller.Vbus_error, controller.Driver_error, controller.Velocity_error,
                          controller.Current_error, controller.ESTOP_error};
    bool Error_array2[] = {controller.Calibrated, controller.Activated, controller.Watchdog_error, 0, 0, 0, 0, 0};

    CAN_TX_msg.data[0] = bitsToByte(Error_array);
    CAN_TX_msg.data[1] = bitsToByte(Error_array2);
    CAN_TX_msg.data[2] = 0x00;
    CAN_TX_msg.data[3] = 0x00;
    CAN_TX_msg.data[4] = 0x00;
    CAN_TX_msg.data[5] = 0x00;
    CAN_TX_msg.data[6] = 0x00;
    CAN_TX_msg.data[7] = 0x00;
    CAN_TX_msg.len = 2;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_IN_STATE_OF_ERRORS_CAN, controller.Error);
    CANSend(&CAN_TX_msg);
    controller.Send_heartbeat = 0;
}



/// @brief Send gripper data packet (4 byte)
/// Direction Driver -> host
/// Pack data in this format:
/// Position 1 byte
/// Current 2 byte
/// 1 byte:
///  bit 0 - gripper activate (1) / deactivated (0)
///  bit 1 Gripper action status - 1 is goto, 0 is idle or performing auto release or in calibration)
///  object detection bit 2 and 3 - 
///  bit 4 - gripper temperature error
///  bit 5 - gripper timeout error
///  bit 6 - gripper estop error
///  bit 7 - gripper calibration status; calibrated (1) / not calibrated (0)
void Gripper_pack_data()
{
    byte data_buffer_send2[2];
    Gripper.current = FOC.Iq;
    intTo2Bytes(Gripper.current, data_buffer_send2);

    // Unpack object detection data (from 0 - 3) into 2 bits
    int data_ = constrain(Gripper.object_detection_status, 0, 3);
    int packedData_ = data_ & 0b11;
    bool object_detection_bit_1 = (packedData_ & (1 << 0)) != 0;
    bool object_detection_bit_2 = (packedData_ & (1 << 1)) != 0;

    bool array_[] = {Gripper.activated,Gripper.action_status,object_detection_bit_1,object_detection_bit_2,
    Gripper.temperature_error,Gripper.timeout_error,Gripper.estop_error,Gripper.calibrated};

    //bool array_[] = {0,1,0,1,0,1,1,1};

    CAN_TX_msg.data[0] = Gripper.position;
    CAN_TX_msg.data[1] = data_buffer_send2[0];
    CAN_TX_msg.data[2] = data_buffer_send2[1];
    CAN_TX_msg.data[3] = bitsToByte(array_);
    CAN_TX_msg.data[4] = 0x00;
    CAN_TX_msg.data[5] = 0x00;
    CAN_TX_msg.data[6] = 0x00;
    CAN_TX_msg.data[7] = 0x00;
    CAN_TX_msg.len = 4;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_GRIPPER_DATA_PACK, controller.Error);
    CANSend(&CAN_TX_msg);
    controller.Send_heartbeat = 0;
}



/// @brief Send motor data pack 1 (8 byte)
/// Direction Driver -> host
///Pack data in this format:
/// Position 3 bytes [Encoder ticks]
/// Speed 3 bytes [Encoder ticks / s]
/// Torque 2 byte [mA]
void Data_pack_1_CAN()
{
    byte data_buffer_send[8] = {0,0,0,0,0,0,0,0};
    packData1(controller.Position_Ticks, controller.Velocity_Filter, FOC.Iq, data_buffer_send);
    //packData1(-150, -187, 3047, data_buffer_send);
    CAN_TX_msg.data[0] = data_buffer_send[0];
    CAN_TX_msg.data[1] = data_buffer_send[1];
    CAN_TX_msg.data[2] = data_buffer_send[2];
    CAN_TX_msg.data[3] = data_buffer_send[3];
    CAN_TX_msg.data[4] = data_buffer_send[4];
    CAN_TX_msg.data[5] = data_buffer_send[5];
    CAN_TX_msg.data[6] = data_buffer_send[6];
    CAN_TX_msg.data[7] = data_buffer_send[7];
    CAN_TX_msg.len = 8;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_DATA_PACK_1, controller.Error);
    CANSend(&CAN_TX_msg);
    controller.Send_heartbeat = 0;
}

/// @brief Send motor data pack 2 (0 byte) Not used in this version
/// Direction Driver -> host
void Data_pack_2_CAN()
{
    CAN_TX_msg.data[0] = 0x00;
    CAN_TX_msg.data[1] = 0x00;
    CAN_TX_msg.data[2] = 0x00;
    CAN_TX_msg.data[3] = 0x00;
    CAN_TX_msg.data[4] = 0x00;
    CAN_TX_msg.data[5] = 0x00;
    CAN_TX_msg.data[6] = 0x00;
    CAN_TX_msg.data[7] = 0x00;
    CAN_TX_msg.len = 0;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_DATA_PACK_2, controller.Error);
    CANSend(&CAN_TX_msg);
    controller.Send_heartbeat = 0;
}

/// @brief Send motor data pack 3 (0 byte) Not used in this version
/// Direction Driver -> host
void Data_pack_3_CAN()
{
    CAN_TX_msg.data[0] = 0x00;
    CAN_TX_msg.data[1] = 0x00;
    CAN_TX_msg.data[2] = 0x00;
    CAN_TX_msg.data[3] = 0x00;
    CAN_TX_msg.data[4] = 0x00;
    CAN_TX_msg.data[5] = 0x00;
    CAN_TX_msg.data[6] = 0x00;
    CAN_TX_msg.data[7] = 0x00;
    CAN_TX_msg.len = 0;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.id = Combine_2_CAN_ID(controller.CAN_ID, OUT_DATA_PACK_3, controller.Error);
    CANSend(&CAN_TX_msg);
    controller.Send_heartbeat = 0;
}

/// @brief Packs position, speed and torque data into 8 bytes
/// @param position 
/// @param speed 
/// @param torque 
/// @param bytes 
void packData1(int position, int speed, int torque, byte *bytes)
{
    // Ensure the input values are within the specified ranges
    // position = constrain(position, -8388608, 8388607); // Assuming a 24-bit signed integer range
    // speed = constrain(speed, -8388608, 8388607); // Assuming a 24-bit signed integer range
    // torque = constrain(torque, 0, 65535); // Assuming a 16-bit unsigned integer range

    // Pack the signed position into the first 3 bytes
    bytes[0] = (position >> 16) & 0xFF;
    bytes[1] = (position >> 8) & 0xFF;
    bytes[2] = position & 0xFF;

    // Pack the signed speed into the next 3 bytes
    bytes[3] = (speed >> 16) & 0xFF;
    bytes[4] = (speed >> 8) & 0xFF;
    bytes[5] = speed & 0xFF;

    // Pack the signed torque into the last 2 bytes
    bytes[6] = (torque >> 8) & 0xFF;
    bytes[7] = torque & 0xFF;
}
