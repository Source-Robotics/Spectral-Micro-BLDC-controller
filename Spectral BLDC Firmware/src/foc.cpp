
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    foc.cpp
  * @brief   This file provides code where we define FOC functions
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

#include "utils.h"
#include "common.h"
#include "cogging.h"
#include <Arduino.h>
#include <stdio.h>
#include "iodefs.h"
#include "hw_init.h"
#include "motor_control.h"
#include "common.h"
#include <SPI.h>
#include "foc.h"
#include "qfplib-m3.h"


/// @brief Space vector commutation 
/// @param v_bus Input V_bus value
/// @param u Input U
/// @param v Input V
/// @param w Input W
/// @param u_normalized Output U
/// @param v_normalized Output V
/// @param w_normalized Output W
void space_vector_commutation(float v_bus, float u, float v, float w, volatile float *u_normalized, volatile float *v_normalized, volatile float *w_normalized){

float offset =  (fminf3(u, v, w) + fmaxf3(u, v, w))/2.0f;
/// Here we normalize our Ua, Ub and Uc voltages that can  span from Vbus to - Vbus; to a range of 0 - 1.
/// This normalization ensures that the calculated values are within the typical 
/// range of duty cycles used in PWM (Pulse Width Modulation), which is commonly [0, 1].
/// Values are normalized around vbus, so if Ua,Ub or Uc go above vbus they need to be clamped.
/// *** In case of space vector Ua, Ub and Uc can GO ABOVE VBUS but still need to be clamped to not go below 0 and above 1.
*u_normalized = ((u - offset) /v_bus + 1.0f) / 2.0f;
*v_normalized = ((v - offset) /v_bus + 1.0f) / 2.0f;
*w_normalized = ((w - offset) /v_bus + 1.0f) / 2.0f;

}

/// @brief Sinusoidal commutation
/// @param v_bus Input V_bus value
/// @param u Input U
/// @param v Input V
/// @param w Input W
/// @param u_normalized Output U
/// @param v_normalized Output V
/// @param w_normalized Output W
void sinusoidal_commutation(float v_bus, float u, float v, float w, volatile float *u_normalized, volatile float *v_normalized, volatile  float *w_normalized){

/// Here we normalize our Ua, Ub and Uc voltages that can  span from Vbus to - Vbus; to a range of 0 - 1.
/// This normalization ensures that the calculated values are within the typical 
/// range of duty cycles used in PWM (Pulse Width Modulation), which is commonly [0, 1].
/// Values are normalized around vbus, so if Ua,Ub or Uc go above vbus they need to be clamped.
*u_normalized = (u/v_bus + 1.0f) / 2.0f;
*v_normalized = (v/v_bus + 1.0f) / 2.0f;
*w_normalized = (w/v_bus + 1.0f) / 2.0f;

}


/// @brief Used for transforming Uq and Ud to Ua Ub and Uc. Before calling dq0_abc_variables needs to be called.
/// @param d Input Ud
/// @param q Input Uq 
/// @param a Output Ua
/// @param b Output Ub
/// @param c Output Uc
void abc_fast(float d, float q, volatile float *a, volatile float *b, volatile float *c){
    /// Inverse DQ0 Transform ///
    FOC.Ud = d; // as asked, for the capture (signals 17, 18)
    FOC.Uq = q;
    d += FOC.sound_d; // the sound (batch 22), on top of what the loops ask
    q += FOC.sound_q;
    float dr = d; // rotated forward by speed x delay (parameter 64); not at all below COMM_DELAY_MIN_SPEED
    float qr = q;
    const float ds = FOC.delay_s;
    if (ds != 0.0f)
    {
      const float dc = FOC.delay_c;
      dr = d * dc - q * ds;
      qr = d * ds + q * dc;
    }
    *a = FOC.cosine_value * dr - FOC.sine_value * qr;                // Faster Inverse DQ0 transform
    *b = (FOC.const1)*dr - (FOC.const4)*qr;
    *c = (FOC.const3)*dr - (FOC.const2)*qr;
    }



/// @brief Used for transforming Uq and Ud to Ua Ub and Uc.
/// @param theta Electrical angle
/// @param d Input Ud
/// @param q Input Uq 
/// @param a Output Ua
/// @param b Output Ub
/// @param c Output Uc
void abc( float theta, float d, float q, float *a, float *b, float *c){
    /// The DQZ transform is the product of the Clarke transform and the Park transform
    /// Inverse DQ0 Transform ///
    /// Phase current amplitude = lengh of dq vector///
    /// i.e. iq = 1, id = 0, peak phase current of 1///
    float cf = qfp_fcos(theta);
    float sf = qfp_fsin(theta);
    
    *a = cf*d - sf*q;                // Faster Inverse DQ0 transform
    *b = (0.86602540378f*sf-.5f*cf)*d - (-0.86602540378f*cf-.5f*sf)*q;
    *c = (-0.86602540378f*sf-.5f*cf)*d - (0.86602540378f*cf-.5f*sf)*q;
    }
    
/// @brief Calculates all variables that are used both in abc and dq0 tranforms, giving a speed boost
/// @param theta Electrical angle
void dq0_abc_variables(float theta){
    if (controller.fast_trig) // the integer quarter-wave table (cogging.cpp), 16384 steps per turn (parameter 65)
    {
        const int32_t a = (int32_t)(theta * 2607.5945f);
        FOC.sine_value = (float)Cogging_sin_q15((uint32_t)a) * 3.0517578e-5f;
        FOC.cosine_value = (float)Cogging_sin_q15((uint32_t)a + 4096) * 3.0517578e-5f;
    }
    else
    {
        FOC.sine_value = qfp_fsin(theta);
        FOC.cosine_value = qfp_fcos(theta);
    }

    FOC.sin_sqrt3div2 = (SQRT3DEV2 * FOC.sine_value); // 0.86602540378f * sine_value
    FOC.cos_sqrt3div2 = (SQRT3DEV2 * FOC.cosine_value); // 0.86602540378f * cosine_value

    FOC.sin_05 = (0.5f * FOC.sine_value); // 0.5 * sin_value
    FOC.cos_05 = (0.5f * FOC.cosine_value); // 0.5 * cos_value

    FOC.const1 = (FOC.sin_sqrt3div2 -  FOC.cos_05);
    FOC.const2 = (FOC.cos_sqrt3div2 -  FOC.sin_05);

    FOC.const3 = (-FOC.sin_sqrt3div2 -  FOC.cos_05);
    FOC.const4 = (-FOC.cos_sqrt3div2 -  FOC.sin_05);
}




/// @brief Used to get Iq and Id from Ia Ib and Ic. Before calling dq0_abc_variables needs to be called.
/// @param a Input Ia
/// @param b Input Ib
/// @param c Input Ic
/// @param d Output Ud
/// @param q Output Uq
void dq0_fast( float a, float b, float c, float *d, float *q){


    *d = 0.6666667f*(FOC.cosine_value * a + (FOC.const1)*b + (FOC.const3)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-FOC.sine_value * a - (FOC.const4)*b - (FOC.const2)*c);
       
    }


/// @brief Same as dq0_fast but inputs are integers.
/// @param a Input Ia
/// @param b Input Ib
/// @param c Input Ic
/// @param d Output Ud
/// @param q Output Uq
void dq0_fast_int( int a, int b, int c, volatile float *d,volatile  float *q){


    *d = 0.6666667f*(FOC.cosine_value * a + (FOC.const1)*b + (FOC.const3)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-FOC.sine_value * a - (FOC.const4)*b - (FOC.const2)*c);
       
    }    

 
/// @brief Used to get Iq and Id from Ia Ib and Ic. 
/// @param theta Electrical angle
/// @param a Input Ia
/// @param b Input Ib
/// @param c Input Ic
/// @param d Output Ud
/// @param q Output Uq
void dq0(float theta, float a, float b, float c, float *d, float *q){
    /// The DQZ transform is the product of the Clarke transform and the Park transform
    /// DQ0 Transform ///
    /// Phase current amplitude = lengh of dq vector///
    /// i.e. iq = 1, id = 0, peak phase current of 1///
    
    float cf = qfp_fcos(theta);
    float sf = qfp_fsin(theta);
    
    *d = 0.6666667f*(cf*a + (0.86602540378f*sf-.5f*cf)*b + (-0.86602540378f*sf-.5f*cf)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-sf*a - (-0.86602540378f*cf-.5f*sf)*b - (0.86602540378f*cf-.5f*sf)*c);
       
    }