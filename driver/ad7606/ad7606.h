#ifndef AD7606_H
#define AD7606_H

#include "./include/DSP2833x_Device.h"
#include "./include/DSP2833x_Examples.h"

#define   AD7606_SET_RANGE      GpioDataRegs.GPASET.bit.GPIO6=1

#define   AD7606_CLR_RST        GpioDataRegs.GPACLEAR.bit.GPIO9=1
#define   AD7606_SET_RST        GpioDataRegs.GPASET.bit.GPIO9=1

#define   AD7606_SET_CONV_A     GpioDataRegs.GPASET.bit.GPIO7=1
#define   AD7606_CLR_CONV_A     GpioDataRegs.GPACLEAR.bit.GPIO7=1

//#define   AD7606_SET_RD         GpioDataRegs.GPASET.bit.GPIO10=1
//#define   AD7606_CLR_RD         GpioDataRegs.GPACLEAR.bit.GPIO10=1

#define   AD7606_BUSY           GpioDataRegs.GPADAT.bit.GPIO10
#define   AD7606_FIRST_DATA     GpioDataRegs.GPADAT.bit.GPIO11

#define   AD7606_DELAY            asm(" RPT #7 || NOP"); // Specific delay time is about?

extern volatile float Result_Buf[8] ;

#define AD7606_REF_VOLTAGE 2.5f
#define AD7606_REF_RESOLUTION   0.000305176f// AD7606_REF_VOLTAGE * 8.0f / 65536.0f

void AD7606_Init(void) ;
void AD7606_Start(void) ;

// If use the XINTF bus to read the AD7606
void AD7606_XINTF_Read_All(void);
// float AD7606_Read(int * channel) ;
void AD7606_PostSampleDo(void);
void filter_AD7606();
#endif
