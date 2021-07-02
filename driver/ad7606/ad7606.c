#include <driver/ad7606/ad7606.h>
#include "setup.h"
#include <others/keys.h>
#include "sensor.h"


float MeasureBuf[8] = {380.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
float MeasureBuf_prev[8] = { 380.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

volatile int MeasureBuf_filter[4] = {0,0,0,0};
volatile int MeasureBuf_filter_before[4] = {0,0,0,0};

int count_ad7606 = 0, i_ad7606 = 0;
volatile int temp[8],temp_filter[8] ;

void filter_AD7606();

void AD7606_Init(void)
{
    EALLOW;
    // GPIO config for AD7606(not using XINTF)
    // If using XINTF, the gpio is initialized by InitXintf()
#if !USE_ADC_XINTF
    // For byte read mode
    GpioCtrlRegs.GPBMUX2.bit.GPIO55=0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO56=0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO57=0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO58=0;

    GpioCtrlRegs.GPAMUX1.bit.GPIO4=0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5=0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO6=0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7=0;


    GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0 ;
    GpioCtrlRegs.GPBDIR.bit.GPIO56 = 0 ;
    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 0 ;
    GpioCtrlRegs.GPBDIR.bit.GPIO58 = 0 ;

    GpioCtrlRegs.GPADIR.bit.GPIO4 = 0 ;
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 0 ;
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 0 ;
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 0 ;
    // End for byte read mode
#endif

    GpioCtrlRegs.GPADIR.bit.GPIO6=1 ;
    GpioCtrlRegs.GPADIR.bit.GPIO7=1 ;
    GpioCtrlRegs.GPADIR.bit.GPIO8=0 ; // Set as input ;
    GpioCtrlRegs.GPADIR.bit.GPIO9=1 ;
    GpioCtrlRegs.GPADIR.bit.GPIO10=0;
    GpioCtrlRegs.GPADIR.bit.GPIO11=0;

    GpioCtrlRegs.GPAMUX1.bit.GPIO6= 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7= 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO8=0 ; // Common function ;
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;

    EDIS;

    AD7606_SET_RANGE ;

    AD7606_CLR_RST ;
    AD7606_DELAY ;

    AD7606_SET_RST ;
    AD7606_DELAY ;

    AD7606_CLR_RST ;
    AD7606_DELAY ;
}

void AD7606_Start(void)
{
    Uint16 i = 0 , j = 1;
    AD7606_CLR_CONV_A ;
    asm(" RPT #9 || NOP");
    AD7606_SET_CONV_A ;
    asm(" RPT #19 || NOP");

    while(AD7606_BUSY)
    {
        // Waiting for end of conversion
        i ++ ;
        // asm(" RPT #1 || NOP");
        if(i > 1000){
            j ++ ;
            return ;
        }

    }

}

//float AD7606_Read(int * channel)
//{
//    int ad16bits = 0 ;
//    static int count = 0 ;
//    AD7606_CLR_CS ;
//    AD7606_CLR_RD ;
//
//    AD7606_DELAY ;
//    AD7606_DELAY ;
//    AD7606_DELAY ;
//
//    ad16bits = ad16bits |((GpioDataRegs.GPBDAT.bit.GPIO55)<<15)|
//                         ((GpioDataRegs.GPBDAT.bit.GPIO56)<<14)|
//                         ((GpioDataRegs.GPBDAT.bit.GPIO57)<<13)|
//                         ((GpioDataRegs.GPBDAT.bit.GPIO58)<<12)|
//                         ((GpioDataRegs.GPADAT.bit.GPIO4)<<11)|
//                         ((GpioDataRegs.GPADAT.bit.GPIO5)<<10)|
//                         ((GpioDataRegs.GPADAT.bit.GPIO6)<<9)|
//                         (GpioDataRegs.GPADAT.bit.GPIO7<<8);
//    AD7606_SET_RD;
//    AD7606_DELAY;
//    AD7606_CLR_RD;
//    AD7606_DELAY ;
//
//    ad16bits = ad16bits|((GpioDataRegs.GPBDAT.bit.GPIO55)<<7)|
//                        ((GpioDataRegs.GPBDAT.bit.GPIO56)<<6)|
//                        ((GpioDataRegs.GPBDAT.bit.GPIO57)<<5)|
//                        ((GpioDataRegs.GPBDAT.bit.GPIO58)<<4)|
//                        ((GpioDataRegs.GPADAT.bit.GPIO4)<<3)|
//                        ((GpioDataRegs.GPADAT.bit.GPIO5)<<2)|
//                        ((GpioDataRegs.GPADAT.bit.GPIO6)<<1)|
//                        (GpioDataRegs.GPADAT.bit.GPIO7);
//    AD7606_SET_RD ;
//
//    *channel = count ;
//    return ad16bits * AD7606_REF_RESOLUTION ;
//}

// If use the XINTF bus to read the AD7606
void AD7606_XINTF_Read_All(void){
    AD7606_Start();
    // SCOPE_PD ;
    // AD7606_CLR_CS ; in GPIO mode
    //asm(" RPT #50 || NOP");

//    for(i = 0; i < 8; i++){
//        temp[i] = * ( (Uint16 *) XINTF_AD7606) ;
//        // asm(" RPT #1 || NOP");
//        if(AD7606_FIRST_DATA){
//            count = i ;
//        }
//    }
    temp[0] = * ( (Uint16 *) XINTF_AD7606) ;
//    if(AD7606_FIRST_DATA) count = i_ad7606 ++ ;
    temp[1] = * ( (Uint16 *) XINTF_AD7606) ;
//    if(AD7606_FIRST_DATA) count = i_ad7606 ++ ;
    temp[2] = * ( (Uint16 *) XINTF_AD7606) ;
//    if(AD7606_FIRST_DATA) count = i_ad7606 ++ ;
    temp[3] = * ( (Uint16 *) XINTF_AD7606) ;
//    if(AD7606_FIRST_DATA) count = i_ad7606 ++ ;
    // temp[4] = * ( (Uint16 *) XINTF_AD7606) ;
//    if(AD7606_FIRST_DATA) count = i_ad7606 ++ ;
    // temp[5] = * ( (Uint16 *) XINTF_AD7606) ;
//    if(AD7606_FIRST_DATA) count = i_ad7606 ++ ;
    // temp[6] = * ( (Uint16 *) XINTF_AD7606) ;
//    if(AD7606_FIRST_DATA) count = i_ad7606 ++ ;
    // temp[7] = * ( (Uint16 *) XINTF_AD7606) ;
//    if(AD7606_FIRST_DATA) count = i_ad7606 ++ ;

    // SCOPE_PU ;
//    for(i = 0 ; i < 4; i ++ ){
//        j = (count + i ) % 8 ;
//        Result_Buf[i] = temp[j] * AD7606_REF_RESOLUTION ;
//    }
    // DC Voltage and Capacitor Current should be calculated first
    filter_AD7606();
    MeasureBuf[CH_DC_BUS] = MeasureBuf_filter[CH_DC_BUS] * DC_VOLTAGE_MUL + DC_MEASURE_OFFSET;
    MeasureBuf[CH_CAP_CURRENT] = MeasureBuf_filter[CH_CAP_CURRENT] * IC_CURRENT_MUL + IC_MEASURE_OFFSET;

    //SCOPE_PD ;
}

void AD7606_PostSampleDo(){
    // AC Voltage and Grid Current can be calculated later ;
    MeasureBuf[CH_AC_VOLTAGE] = MeasureBuf_filter[(count_ad7606 + CH_AC_VOLTAGE) % 8] * AC_VOLTAGE_MUL + AC_MEASURE_OFFSET;
    MeasureBuf[CH_GRID_CURRENT] = MeasureBuf_filter[(count_ad7606 + CH_GRID_CURRENT) % 8] * IG_CURRENT_MUL + IG_MEASURE_OFFSET ;
}


//AD7606采样的低通滤波器设计
void filter_AD7606(){
    Uint16 i = 0 ;
    Uint16 frequency = 10000;
    Uint16 alpha = 500;
    for(i = 0 ; i < 4; i ++){
        MeasureBuf_filter[i] = temp[i] * alpha + frequency * MeasureBuf_filter_before[i];
        MeasureBuf_filter[i] = (int) MeasureBuf_filter[i] /(frequency + alpha);
        MeasureBuf_filter_before[i] = MeasureBuf_filter[i];
    }
}
