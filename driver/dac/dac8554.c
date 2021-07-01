/*
Date:   2017-6-30
Author: Donghao@Lab209, NUAA
*/

#include <driver/dac/dac8554.h>

void DAC8554Init()
{
#if !USE_DAC_SPI
    EALLOW ;
    GpioCtrlRegs.GPBMUX2.bit.GPIO54=0; // IO
    GpioCtrlRegs.GPBDIR.bit.GPIO54=1; // MOSI output

    GpioCtrlRegs.GPBMUX2.bit.GPIO56=0; // IO
    GpioCtrlRegs.GPBDIR.bit.GPIO56=1; // SCLK output

    GpioCtrlRegs.GPBMUX2.bit.GPIO57=0; // IO
    GpioCtrlRegs.GPBDIR.bit.GPIO57=1; // SYNC, CS  outnput

	GpioCtrlRegs.GPAMUX1.bit.GPIO12=0; // IO
    GpioCtrlRegs.GPADIR.bit.GPIO12=1; // ENABLE outnput
    
	EDIS ;
    SYNC_DAC8554(1); // Just in case for the first time
    ENABLE_DAC8554(0); // Enable low, active 
	SCLK_DAC8554(1);
#else
	// TODO:
	// SPI hardware communication is enabled
	// 1. GPIO init
	// 2. SPI init
	// 3. DAC init (Actually, nothing to do)

    EALLOW ;


    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0 ; // IO
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1 ; // ENABLE output

    EDIS ;

	DAC8554SpiInit(); // For SPI interface

	GpioDataRegs.GPACLEAR.bit.GPIO12=1 ; // logic low to Enable.


#endif
}

#if !USE_DAC_SPI
/// Write 24 Bits to SPI Bus
void WriteBits(unsigned long dataBits)
{
#if !USE_DAC_SPI
    int count = 0 ;
    SYNC_DAC8554(0);
    for(count = 0 ; count < 24 ; count ++)
    {
        SCLK_DAC8554(1) ;
		// MSB First.
        DIN_DAC8554(dataBits >> 23) ;
        SCLK_DAC8554(0) ;
        dataBits <<= 1 ;
    }
    SYNC_DAC8554(1);
#else
    // TODO:
    // SPI hardware communication is enabled

#endif
}

void StoreVoltage(  unsigned int ch, 
                    float voltage,
                    unsigned char addr, 
                    unsigned char update_flag)
{
    unsigned long controlBits = 0 ;
    unsigned long dataBits = 0 ;
    unsigned long dacBits = voltage / REF_VOLTAGE * 0xffff ;

    controlBits = (addr << 6) | (ch << 1) ;

    if(update_flag > 0){
        controlBits |= 0x20 ; // Simultaneously Update the DAC
    }
    
    dataBits = (controlBits << 16 ) | dacBits ;

    WriteBits(dataBits) ;
}

void DAC8554Output(float va, float vb, float vc, float vd)
{
    StoreVoltage(0, va, ADDR_DAC8554, 0);
    StoreVoltage(1, vb, ADDR_DAC8554, 0);
    StoreVoltage(2, vc, ADDR_DAC8554, 0);
    // Output the voltage simultaneously
    StoreVoltage(3, vd, ADDR_DAC8554, 1);
}
#else
    Uint16 SPI_transfer_buffer[12] ;

void SPIBufferUpdate(float va, float vb, float vc, float vd, unsigned char addr){
    Uint16 i = 0, dacBits ;
    for(i = 0; i < 12; i ++){
        SPI_transfer_buffer[i] = 0 ;
    }
    // Control bytes:
    SPI_transfer_buffer[0] |= CTRLBYTE_0 << 8 ;
    SPI_transfer_buffer[1] = CTRLBYTE_1  ;
    SPI_transfer_buffer[3] |= CTRLBYTE_2 << 8 ;
    SPI_transfer_buffer[4] = CTRLBYTE_3U ;

    dacBits = va / REF_VOLTAGE * 0xffff ;
    SPI_transfer_buffer[0] |= (dacBits >> 8) ;
    SPI_transfer_buffer[1] |= (dacBits << 8) ;

    dacBits = vb / REF_VOLTAGE * 0xffff ;
    SPI_transfer_buffer[2] |= dacBits ;

    dacBits = vc / REF_VOLTAGE * 0xffff ;
    SPI_transfer_buffer[3] |= (dacBits >> 8) ;
    SPI_transfer_buffer[4] |= (dacBits << 8) ;

    dacBits = vd / REF_VOLTAGE * 0xffff ;
    SPI_transfer_buffer[5] |= dacBits ;
    for(i = 0; i < 12; i ++){
        SpiaRegs.SPITXBUF = SPI_transfer_buffer[i];
    }
}

void DAC8554SpiSetVoltage(float v, unsigned char ch, unsigned char update ){
    Uint16 controlBits = 0 ;
    Uint16 dacBits = v / REF_VOLTAGE * 0xffff ;

    controlBits =  ch << 1 ;
    if(update > 0){
        controlBits |= 0x20 ; // Simultaneously Update the DAC
    }
    SpiaRegs.SPITXBUF = ((controlBits << 8) | (dacBits >> 8)) ;
    SpiaRegs.SPITXBUF = (dacBits << 8) ;
}

void DAC8554SpiInit(void){
    // FIFO TX
    SpiaRegs.SPIFFCT.all=0x00;
    SpiaRegs.SPIFFTX.all=0xE040;

    // SPIa
    SpiaRegs.SPICCR.all =0x000B;    // Reset on, rising edge, 12-bit char bits
    SpiaRegs.SPICTL.all =0x0006;    // Enable master mode, normal phase,
                                    // enable talk, and SPI int disabled.

    SpiaRegs.SPIBRR =0x0004;        //150M/2/(BRR+1) = 15M
    SpiaRegs.SPICCR.all =0x009B;    // Relinquish SPI from Reset
    SpiaRegs.SPIPRI.bit.FREE = 1;   // Set so breakpoints don't disturb xmission

    EALLOW ;
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;   // Enable pull-up on GPIO54 (SPISIMOA)
    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;   // Enable pull-up on GPIO56 (SPICLKA)
    GpioCtrlRegs.GPBPUD.bit.GPIO57 = 0;   // Enable pull-up on GPIO57 (SPISTEA)

    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 3; // Asynch input GPIO54 (SPISIMOA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO56 = 3; // Asynch input GPIO56 (SPICLKA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO57 = 3; // Asynch input GPIO57 (SPISTEA)

    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 1; // Configure GPIO54 as SPISIMOA
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 1; // Configure GPIO56 as SPICLKA
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 1; // Configure GPIO57 as SPISTEA

    EDIS;
}
#endif

