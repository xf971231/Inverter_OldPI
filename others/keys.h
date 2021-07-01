/*
 * keys.h
 *
 *  Created on: 2018Äê3ÔÂ13ÈÕ
 *      Author: donghao
 */

#ifndef OTHERS_KEYS_H_
#define OTHERS_KEYS_H_

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

#define KEY_CLOSELOOP_CONTROL_ENABLED (GpioDataRegs.GPADAT.bit.GPIO13==0)
#define KEY_CLOSELOOP_CONTROL_DISABLED (GpioDataRegs.GPADAT.bit.GPIO20==0)

#define SCOPE_TOGGLE   {GpioDataRegs.GPBTOGGLE.bit.GPIO61=1;}
#define SCOPE_PU    {GpioDataRegs.GPBSET.bit.GPIO61=1;}
#define SCOPE_PD    {GpioDataRegs.GPBCLEAR.bit.GPIO61=1;}

//#define COL_CLEAR_0 {GpioDataRegs.GPACLEAR.bit.GPIO21=1;}
//#define COL_CLEAR_1 {GpioDataRegs.GPACLEAR.bit.GPIO20=1;}
//#define COL_CLEAR_2 {GpioDataRegs.GPACLEAR.bit.GPIO27=1;}
//
//#define COL_SET_0 {GpioDataRegs.GPASET.bit.GPIO21=1;}
//#define COL_SET_1 {GpioDataRegs.GPASET.bit.GPIO20=1;}
//#define COL_SET_2 {GpioDataRegs.GPASET.bit.GPIO27=1;}
//
//#define ROW_READ_0 GpioDataRegs.GPADAT.bit.GPIO26==0
//#define ROW_READ_1 GpioDataRegs.GPADAT.bit.GPIO25==0
//#define ROW_READ_2 GpioDataRegs.GPADAT.bit.GPIO24==0
//#define ROW_READ_3 GpioDataRegs.GPBDAT.bit.GPIO48==0

void KeyGpioInit(void);

// int KeyScan(void);

#endif /* OTHERS_KEYS_H_ */
