/*
 * relay.c
 *
 *  Created on: 2018Äê3ÔÂ15ÈÕ
 *      Author: donghao
 */

#include <sensor.h>
#include <others/relay_and_protect.h>

volatile float MEASURE_MAX[8];
volatile float CHECK_MAX[8];
volatile int error_counter[8] =  {0, 0, 0, 0, 0, 0} ;
volatile int period_counter = 0 ;

// defined in ADC_Sampling.h
//#define CH_DC_BUS 0
//#define CH_AC_VOLTAGE 1
//#define CH_GRID_CURRENT 2
//#define CH_CAP_CURRENT 3

void RelayGpioInit(void){
    EALLOW ;

    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1 ; // Output
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1 ; // Output
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1 ; // Output


    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0 ; // Common function
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0 ; // Common function
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0 ; // Common function


    EDIS ;

    PWM_DIS ;
    RELAY_1_OPEN ;
    RELAY_2_OPEN ;


    MEASURE_MAX[CH_DC_BUS] = DC_VOLTAGE_MAX ;
    MEASURE_MAX[CH_AC_VOLTAGE] = AC_VOLTAGE_MAX ;
    MEASURE_MAX[CH_GRID_CURRENT] = IF_CURRENT_MAX ;
    MEASURE_MAX[CH_CAP_CURRENT] = IC_CURRENT_MAX ;

#if LONG_TIME_RUNNING_TEST
    CHECK_MAX[CH_DC_BUS] = DC_VOLTAGE_CHECK_MAX ;
    CHECK_MAX[CH_AC_VOLTAGE] = AC_VOLTAGE_CHECK_MAX ;
    CHECK_MAX[CH_GRID_CURRENT] = IF_CURRENT_CHECK_MAX ;
    CHECK_MAX[CH_CAP_CURRENT] = IC_CURRENT_CHECK_MAX ;
#endif
}

void Shutdown_PWM_RELAY(void){
    RELAY_1_OPEN ;
    RELAY_2_OPEN ;
    PWM_DIS ;
}

void Enable_PWM_RELAY(void){
    PWM_EN ;
    RELAY_2_CLOSEUP ;
    RELAY_1_CLOSEUP ;
}

int ErrorDetected(void){
    int i = 0 ;
    int error_sum = 0 ;

    error_counter[period_counter] = 0;

    for(i = 0 ; i < 4 ; i ++){
        if(MeasureBuf[i] > MEASURE_MAX[i]){
            error_counter[period_counter] ++ ;
            MeasureBuf[i] = MeasureBuf_prev[i] ;
        }
        else if(MeasureBuf[i] < -MEASURE_MAX[i]){
            error_counter[period_counter] ++ ;
            MeasureBuf[i] = MeasureBuf_prev[i] ;
        }else{
            MeasureBuf_prev[i] = MeasureBuf[i] ;
        }
    }
    period_counter ++ ;
    period_counter %= 8 ;

    for(i = 0 ; i < 8 ; i ++){
            error_sum += error_counter[i] ;
    }

    if( error_sum >=7 ){
        return 1 ;
    }
    else{
        return 0 ;
    }
}

#if LONG_TIME_RUNNING_TEST

void MeasureCheck(void){
    int i = 0 ;
    for(i = 0 ; i < 4 ; i ++)
    {
        if(MeasureBuf[i] > CHECK_MAX[i]){
            flag_measure_check |= (1<< i) ;
        }
        else if(MeasureBuf[i] < -CHECK_MAX[i]){
            flag_measure_check |= (1<< i) ;
        }
    }
}

#endif
