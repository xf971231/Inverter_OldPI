#ifndef DAC8554_H
#define DAC8554_H

#include "../../setup.h"

// Change the GPIO macro definition here 
#define SCLK_DAC8554(x)     {GpioDataRegs.GPBDAT.bit.GPIO56=x;}
#define SYNC_DAC8554(x)     {GpioDataRegs.GPBDAT.bit.GPIO57=x;}
#define DIN_DAC8554(x)      {GpioDataRegs.GPBDAT.bit.GPIO54=x;}
// MISO is not used in this case: it's better to pull down the ENABLE to GND;
#define ENABLE_DAC8554(x)   {GpioDataRegs.GPADAT.bit.GPIO12=x;}
#define DAC8554_DALAY       {sm(" RPT #7 || NOP");}


#define REF_VOLTAGE 5.0 

// The address of DAC8554 
#define ADDR_DAC8554 0b00
#define ADDR_LEFTSHIFT6 0x00 // For ADDR_DAC8554 = 0b00

#define CTRLBYTE_0 ((ADDR_DAC8554 << 6) | (0 << 1))
#define CTRLBYTE_1 ((ADDR_DAC8554 << 6) | (0 << 1))
#define CTRLBYTE_2 ((ADDR_DAC8554 << 6) | (0 << 1))
#define CTRLBYTE_3 ((ADDR_DAC8554 << 6) | (0 << 1))
#define CTRLBYTE_3U ((ADDR_DAC8554 << 6) | (0 << 1) | 0x20)

void DAC8554Init(void);
#if !USE_DAC_SPI
void WriteBits(unsigned long dataBit);
void StoreVoltage(unsigned int ch, float voltage, unsigned char addr, unsigned char flag);
void DAC8554Output(float va, float vb, float vc, float vd);
#else
void SPIBufferUpdate(float va, float vb, float vc, float vd, unsigned char addr);
void DAC8554SpiInit(void);
void DAC8554SpiSetVoltage(float v, unsigned char ch, unsigned char update );
#endif

#endif
