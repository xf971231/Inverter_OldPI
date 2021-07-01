/*
 * keys.c
 *
 *  Created on: 2018Äê3ÔÂ13ÈÕ
 *      Author: donghao
 */

#include "keys.h"

void KeyGpioInit(void){
    EALLOW ;
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 0 ; // Set as input ;
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0 ; // Set as common function

    GpioCtrlRegs.GPADIR.bit.GPIO20 = 0 ; // Set as input ;
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0 ; // Set as common function

    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1 ; // Set as output
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0 ; // Set as common function

    EDIS ;
//    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1 ;
//    GpioCtrlRegs.GPADIR.bit.GPIO21 = 1 ;
//    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1 ;
//
//    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0 ;
//    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0 ;
//    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0 ;
//
//    GpioCtrlRegs.GPADIR.bit.GPIO24 = 0 ;
//    GpioCtrlRegs.GPADIR.bit.GPIO25 = 0 ;
//    GpioCtrlRegs.GPADIR.bit.GPIO26 = 0 ;
//    GpioCtrlRegs.GPBDIR.bit.GPIO48 = 0 ;
//
//    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0 ;
//    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0 ;
//    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0 ;
//    GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0 ;

}

//int KeyScan(void){
//
//
//    if(ROW_READ_0){
//        return 1 ;
//    }
//    if(ROW_READ_1){
//        return 1 ;
//    }
//    if(ROW_READ_2){
//        return 1 ;
//    }
//    if(ROW_READ_3){
//        return 1 ;
//    }
//
//    return 0 ;
//    COL_SET_0 ;
//    COL_SET_1 ;
//    COL_SET_2 ;
//
//}
