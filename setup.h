/*
 * Inverter_Setup.h
 *
 *  Created on: 2017.12.15
 *      Author: donghao
 */
#include "./include/DSP2833x_Device.h"
#include "./include/DSP2833x_Examples.h"

#ifndef INVERTER_SETUP_H_
#define INVERTER_SETUP_H_

// Define switching frequency
#define FREQUENCY_SWITCHING 10000
#define FREQUENCY_SYSSCLK 150000000

#define FAKE_GRID 0

// Define address of AD7606
#define XINTF_ZONE_0 0x004000 // zone 0
#define XINTF_ZONE_6 0x100000 // zone 6
#define XINTF_ZONE_7 0x200000 // zone 7

#define XINTF_AD7606 (XINTF_ZONE_0+1)

//enum InverterOutputState { InitialState, VoltageControlled, CurrentControlledPreparation, CurrentControlled, ErrorEncountered};
enum InverterOutputState { InitialState, VoltageControlled, ControlRelay, ControlPWMOE, CurrentControlled,ErrorEncountered};
/*************** Open Loop Test *****************************/
#define OPENLOOP_TEST 0

// USE_AD7606_OR_56
// 1 : Use AD7606
// 0 : Use AD7656
#define USE_AD7606_OR_56 1

extern int flag_timer2_updated;
extern float64 CarrierWave ;


/****************** macro definition ***********************/
#define USE_ADC 1
#define USE_ADC_XINTF 1

// TODO : DMA can be used to read data from ADC via XINTF
#define USE_ADC_XINTF_DMA 0

#define USE_DAC 1
#if USE_DAC
#define USE_DAC_SPI 0  // SPI hardware communication is disabled by default
#endif

#define USE_PWM 1
#if USE_PWM
#define USE_BIPOLAR 1 // Bipolar modulation is enabled by default

// TODO : Unipolar modulation is not recommended (by Donghao 2018.02.28)
#if USE_BIPOLAR
    #define USE_UNIPOLAR 0
#else
    #define USE_UNIPOLAR 1
#endif
#endif



#endif /* INVERTER_SETUP_H_ */
