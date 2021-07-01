/*
 * relay.h
 *
 *  Created on: 2018Äê3ÔÂ15ÈÕ
 *      Author: donghao
 */

#ifndef OTHERS_RELAY_AND_PROTECT_H_
#define OTHERS_RELAY_AND_PROTECT_H_

#include "../include/DSP2833x_Device.h"
#include "../include/DSP2833x_Examples.h"
#include "../setup.h"

#include "ltrt.h"

#define PWM_EN {GpioDataRegs.GPBSET.bit.GPIO52=1;}
#define PWM_DIS  {GpioDataRegs.GPBCLEAR.bit.GPIO52=1;}

#define RELAY_1_OPEN {GpioDataRegs.GPBSET.bit.GPIO32=1;}
#define RELAY_2_OPEN {GpioDataRegs.GPBSET.bit.GPIO33=1;}


#define RELAY_1_CLOSEUP {GpioDataRegs.GPBCLEAR.bit.GPIO32=1;}
#define RELAY_2_CLOSEUP {GpioDataRegs.GPBCLEAR.bit.GPIO33=1;}


void RelayGpioInit(void) ;
void Shutdown_PWM_RELAY(void) ;
int ErrorDetected(void);
void Enable_PWM_RELAY(void);

#ifdef LONG_TIME_RUNNING_TEST
void MeasureCheck(void) ;
#endif

#endif /* OTHERS_RELAY_AND_PROTECT_H_ */
