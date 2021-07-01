/*
 * control_para.h
 *
 *  Created on: 2018Äê3ÔÂ12ÈÕ
 *      Author: donghao
 */

#ifndef CONTROLLER_CONTROL_PARA_H_
#define CONTROLLER_CONTROL_PARA_H_

/***************************************************************************************************/
// #define VOLTAGE_REF 80.0f
#define CURRENT_REF 10.0f
// #define FREQ_G 50.0f

#define KP_PLL 0.2
#define KI_PLL 0.6
#define K_GAIN_SOGI_PLL 1.4142135623730950488016887242097

// #define TS_PLL_TEN_KHZ 1e-4
#define TS_TWENTY_KHZ 5e-5

#define K_FEEDFORWARD 0.0

// L = 6.0mH
//#define C_CTRL_KP 15.0f
//#define C_CTRL_KI 90000.0f

// L1 = 3.87 mH,
// C = 10 uF
// L2 = 2.2 mH
#define C_CTRL_KP 20.0f
#define C_CTRL_KI 15000.0f
#define C_CTRL_KD 25.0f

// #define V_CTRL_KP 0.02f
// #define V_CTRL_KI 60.0f

#define RC_LEAD_STEPS 11
#define RC_Q_COEFF 0.9
#define RC_K_RC 1.0

extern const float num_filter[3] ;
extern const float den_filter[3] ;
extern const int order_filter ;


#endif /* CONTROLLER_CONTROL_PARA_H_ */
