/*
 * pwm.c
 *
 *  Created on: 2017Äê12ÔÂ18ÈÕ
 *      Author: donghao
 */


#include <driver/ad7606/ad7606.h>
#include <driver/pwm/pwm.h>
#include <setup.h>
#include "../../others/keys.h"
#include "algorithm/control_para.h"

volatile Uint16 CarrierAmplitude = 0 ;
volatile Uint16 CarrierPRD = 0 ;
float vm = 0.0 ;
float damping = 0.0 ;
Uint16 CMP ;

extern float control_modulation ;
extern float MeasureBuf[8];

#if OPENLOOP_TEST
    float sin_test = 0 ;
    int sin_test_count ;
#endif

int flag_timer2_updated = 0 ;
float u = 0 ;


#if UNIPOLAR_MODULATION
interrupt void epwm1_timer_isr(void) // 7500
{
    /***** 7.1us:START *****/
    // SCOPE_PU ;

    AD7606_XINTF_Read_All();
    damping = C_CTRL_KD * MeasureBuf[CH_CAP_CURRENT];
    u = control_modulation - damping ;
    vm = u / MeasureBuf[CH_DC_BUS] ; // vm = -0.7 ;
    if(vm >= 0){
        CMP =  CarrierPRD *  vm ;
        if(CMP > MODULATION_UPPER_THRESHOLD_UINT){
            CMP = MODULATION_UPPER_THRESHOLD_UINT ;
        }
        EPwm1Regs.CMPA.half.CMPA = CMP ;
        EPwm2Regs.CMPA.half.CMPA = 0 ;
    }else{
        vm = - vm ;
        CMP =  CarrierPRD *  vm ;
        if(CMP > MODULATION_UPPER_THRESHOLD_UINT){
            CMP = MODULATION_UPPER_THRESHOLD_UINT ;
        }
        EPwm1Regs.CMPA.half.CMPA = 0 ;
        EPwm2Regs.CMPA.half.CMPA = CMP ;
    }
    // SCOPE_PD ;
    /***** 7.1us:END *****/

    flag_timer2_updated = 1 ;
    AD7606_PostSampleDo();

    EPwm1Regs.ETCLR.bit.INT=1;
    PieCtrlRegs.PIEACK.all=PIEACK_GROUP3;
}

interrupt void epwm2_timer_isr(void) // 0
{
    /***** 7.1us:START *****/
    // SCOPE_PU ;

    AD7606_XINTF_Read_All();
    damping = C_CTRL_KD * MeasureBuf[CH_CAP_CURRENT];
    u = control_modulation - damping ;
    vm = u / MeasureBuf[CH_DC_BUS] ; // vm = -0.7 ;
    if(vm >= 0){
        CMP =  CarrierPRD *  vm ;
        if(CMP > MODULATION_UPPER_THRESHOLD_UINT){
            CMP = MODULATION_UPPER_THRESHOLD_UINT ;
        }
        EPwm1Regs.CMPA.half.CMPA = CMP ;
        EPwm2Regs.CMPA.half.CMPA = 0 ;
    }else{
        vm = - vm ;
        CMP =  CarrierPRD *  vm ;
        if(CMP > MODULATION_UPPER_THRESHOLD_UINT){
            CMP = MODULATION_UPPER_THRESHOLD_UINT ;
        }
        EPwm1Regs.CMPA.half.CMPA = 0 ;
        EPwm2Regs.CMPA.half.CMPA = CMP ;
    }
    // SCOPE_PD ;
    /***** 7.1us:END *****/

    flag_timer2_updated = 1 ;
    AD7606_PostSampleDo();

    EPwm2Regs.ETCLR.bit.INT=1;
    PieCtrlRegs.PIEACK.all=PIEACK_GROUP3;
}


void PWM_Init(int freq)
{
    InitEPwm1Gpio();
    InitEPwm2Gpio();

    // Clear TBCLKSYNC before setup EPwm ;
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=0;
    EDIS;

    Uint32 Reg_TBPRD = FREQUENCY_SYSSCLK/(freq * 2) ;
    CarrierPRD = Reg_TBPRD ;

    // Setup EPWM 1
    EPwm1Regs.TBPRD = Reg_TBPRD - 1 ;
    EPwm1Regs.TBPHS.half.TBPHS = 0;
    EPwm1Regs.TBCTR = 0;

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1 ;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1 ;
    EPwm1Regs.TBCTL.bit.PRDLD = CC_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; //CC_CTR_ZERO
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CAU = AQ_SET ;
    EPwm1Regs.AQCTLB.bit.CAD = AQ_CLEAR ;

    EPwm1Regs.DBCTL.bit.OUT_MODE= DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL=DB_ACTV_HIC;
    EPwm1Regs.DBFED=0;
    EPwm1Regs.DBRED=0;

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;
    EPwm1Regs.ETSEL.bit.INTEN = 1;
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST; // 1 TBPRD triggers 1 interrupt

    // EPWM 2
    EPwm2Regs.TBPRD = Reg_TBPRD - 1 ;
    EPwm2Regs.TBPHS.half.TBPHS = 0;
    EPwm2Regs.TBCTR = 0;

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN ;
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE ;
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_UP ;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1 ;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1 ;
    EPwm2Regs.TBCTL.bit.PRDLD = CC_SHADOW ;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN ;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD ; // CC_CTR_ZERO_PRD ; // Load on Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD ;

    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm2Regs.AQCTLB.bit.CAU = AQ_SET ;
    EPwm2Regs.AQCTLB.bit.CAD = AQ_CLEAR ;

    EPwm2Regs.DBCTL.bit.OUT_MODE= DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL= DB_ACTV_HIC;
    EPwm2Regs.DBFED=0;
    EPwm2Regs.DBRED=0;

    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO ; // when TBCTR=0x0000, the interrupt triggers ;
    EPwm2Regs.ETSEL.bit.INTEN = 1;
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST ; // 1 TBPRD triggers 1 interrupt

    // Set TBCLKSYNC after setup TBCLKSYNC
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=1;
    EDIS;

    EALLOW;
    PieVectTable.EPWM1_INT= &epwm1_timer_isr;
    PieVectTable.EPWM2_INT= &epwm2_timer_isr;
    EDIS;
}

#else

#if UNIPOLAR_MODULATION_2
interrupt void epwm1_timer_isr(void) // 7500
{
    /***** 7.1us:START *****/
    SCOPE_PU ;

    AD7606_XINTF_Read_All();
    damping = C_CTRL_KD * MeasureBuf[CH_CAP_CURRENT];
    u = control_modulation - damping ;
    vm = u / MeasureBuf[CH_DC_BUS] ;

    CMP =  CarrierPRD + CarrierPRD *  vm - 1;
    if(CMP > MODULATION_UPPER_THRESHOLD_UINT){
        CMP = MODULATION_UPPER_THRESHOLD_UINT ;
    }else if(CMP < MODULATION_LOWER_THRESHOLD_UINT ){
        CMP = MODULATION_LOWER_THRESHOLD_UINT ;
    }
    EPwm1Regs.CMPA.half.CMPA = CMP ;
    EPwm2Regs.CMPA.half.CMPA = CarrierPRD * 2 - CMP ;

    SCOPE_PD ;
    /***** 7.1us:END *****/

    flag_timer2_updated = 1 ;
    AD7606_PostSampleDo();

    EPwm1Regs.ETCLR.bit.INT=1;
    PieCtrlRegs.PIEACK.all=PIEACK_GROUP3;
}

interrupt void epwm2_timer_isr(void) // 0
{
    /***** 7.1us:START *****/
    SCOPE_PU ;

    AD7606_XINTF_Read_All();
    damping = C_CTRL_KD * MeasureBuf[CH_CAP_CURRENT];
    u = control_modulation - damping ;
    vm = u / MeasureBuf[CH_DC_BUS] ;
    CMP =  CarrierPRD + CarrierPRD *  vm - 1 ;
    if(CMP > MODULATION_UPPER_THRESHOLD_UINT){
        CMP = MODULATION_UPPER_THRESHOLD_UINT ;
    }else if(CMP < MODULATION_LOWER_THRESHOLD_UINT ){
        CMP = MODULATION_LOWER_THRESHOLD_UINT ;
    }
    EPwm1Regs.CMPA.half.CMPA = CMP ;
    EPwm2Regs.CMPA.half.CMPA = CarrierPRD * 2 - CMP ;
    SCOPE_PD ;
    /***** 7.1us:END *****/

    flag_timer2_updated = 1 ;
    AD7606_PostSampleDo();

    EPwm2Regs.ETCLR.bit.INT=1;
    PieCtrlRegs.PIEACK.all=PIEACK_GROUP3;
}


void PWM_Init(int freq)
{
    InitEPwm1Gpio();
    InitEPwm2Gpio();

    // Clear TBCLKSYNC before setup EPwm ;
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=0;
    EDIS;

    Uint32 Reg_TBPRD = FREQUENCY_SYSSCLK/(freq * 2) ;
    CarrierPRD = Reg_TBPRD /2 ;

    // Setup EPWM 1
    EPwm1Regs.TBPRD = Reg_TBPRD - 1 ;
    EPwm1Regs.TBPHS.half.TBPHS = 0;
    EPwm1Regs.TBCTR = 0;

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1 ;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1 ;
    EPwm1Regs.TBCTL.bit.PRDLD = CC_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; //CC_CTR_ZERO
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
//    EPwm1Regs.AQCTLB.bit.CAU = AQ_SET ;
//    EPwm1Regs.AQCTLB.bit.CAD = AQ_CLEAR ;

    EPwm1Regs.DBCTL.bit.OUT_MODE= DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL=DB_ACTV_HIC;
    EPwm1Regs.DBFED=0;
    EPwm1Regs.DBRED=0;

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;
    EPwm1Regs.ETSEL.bit.INTEN = 1;
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST; // 1 TBPRD triggers 1 interrupt

    // EPWM 2
    EPwm2Regs.TBPRD = Reg_TBPRD - 1 ;
    EPwm2Regs.TBPHS.half.TBPHS = 0;
    EPwm2Regs.TBCTR = 0;

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN ;
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE ;
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_UP ;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1 ;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1 ;
    EPwm2Regs.TBCTL.bit.PRDLD = CC_SHADOW ;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN ;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; //CC_CTR_ZERO
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    EPwm2Regs.AQCTLA.bit.CAU =  AQ_CLEAR ;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET ;

    EPwm2Regs.DBCTL.bit.OUT_MODE= DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL=DB_ACTV_HIC;
    EPwm2Regs.DBFED=0;
    EPwm2Regs.DBRED=0;

    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO ; // when TBCTR=0x0000, the interrupt triggers ;
    EPwm2Regs.ETSEL.bit.INTEN = 1;
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST ; // 1 TBPRD triggers 1 interrupt

    // Set TBCLKSYNC after setup TBCLKSYNC
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=1;
    EDIS;

    EALLOW;
    PieVectTable.EPWM1_INT= &epwm1_timer_isr;
    PieVectTable.EPWM2_INT= &epwm2_timer_isr;
    EDIS;
}
#endif

#if BIPOLAR_MODULATION
interrupt void epwm1_timer_isr(void) // 7500
{
    /***** 7.1us:START *****/
    SCOPE_PU ;

    AD7606_XINTF_Read_All();
    damping = C_CTRL_KD * MeasureBuf[CH_CAP_CURRENT];
    u = control_modulation - damping ;
    vm = - u / MeasureBuf[CH_DC_BUS] ;
    if(vm > 0.95){
        vm = 0.95 ;
    }else if (vm < -0.95){
        vm = -0.95 ;
    }
    CMP =  CarrierAmplitude + CarrierAmplitude *  vm ;

    if(CMP > MODULATION_UPPER_THRESHOLD_UINT){
        CMP = MODULATION_UPPER_THRESHOLD_UINT ;
    }else if(CMP < MODULATION_LOWER_THRESHOLD_UINT ){
        CMP = MODULATION_LOWER_THRESHOLD_UINT ;
    }

    EPwm1Regs.CMPA.half.CMPA = CMP ;
    EPwm2Regs.CMPA.half.CMPA = CMP ;

    SCOPE_PD ;
    /***** 7.1us:END *****/

    //flag_timer2_updated = 1 ;
    AD7606_PostSampleDo();

    EPwm1Regs.ETCLR.bit.INT=1;
    PieCtrlRegs.PIEACK.all=PIEACK_GROUP3;
}

interrupt void epwm2_timer_isr(void) // 0
{
    /***** 7.1us:START *****/
    SCOPE_PU ;

    AD7606_XINTF_Read_All();
    damping = C_CTRL_KD * MeasureBuf[CH_CAP_CURRENT];
    u = control_modulation - damping ;
    vm = - u / MeasureBuf[CH_DC_BUS] ;
    if(vm > 0.95){
        vm = 0.95 ;
    }else if (vm < -0.95){
        vm = -0.95 ;
    }
    CMP =  CarrierAmplitude + CarrierAmplitude *  vm ;

    if(CMP > MODULATION_UPPER_THRESHOLD_UINT){
        CMP = MODULATION_UPPER_THRESHOLD_UINT ;
    }else if(CMP < MODULATION_LOWER_THRESHOLD_UINT ){
        CMP = MODULATION_LOWER_THRESHOLD_UINT ;
    }

    EPwm1Regs.CMPA.half.CMPA = CMP ;
    EPwm2Regs.CMPA.half.CMPA = CMP ;

    SCOPE_PD ;
    /***** 7.1us:END *****/

    flag_timer2_updated = 1 ;
    AD7606_PostSampleDo();

    EPwm2Regs.ETCLR.bit.INT=1;
    PieCtrlRegs.PIEACK.all=PIEACK_GROUP3;
}


void PWM_Init(int freq)
{
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    // Clear TBCLKSYNC before setup EPwm ;
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=0;
    EDIS;

    Uint32 Reg_TBPRD = FREQUENCY_SYSSCLK/(freq * 2) ;
    CarrierAmplitude = Reg_TBPRD / 2 ;

    // Setup EPWM 1
    EPwm1Regs.TBPRD = Reg_TBPRD - 1 ;
    EPwm1Regs.TBPHS.half.TBPHS = 0;
    EPwm1Regs.TBCTR = 0;

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1 ;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1 ;
    EPwm1Regs.TBCTL.bit.PRDLD = CC_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; //CC_CTR_ZERO
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;  // AQ_SET          // Set PWM1A on event A, up count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count
    EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION ;
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION ;
    EPwm1Regs.AQCTLA.bit.CBU = AQ_CLEAR ;
    EPwm1Regs.AQCTLA.bit.CBD = AQ_SET ;

    EPwm1Regs.DBCTL.bit.OUT_MODE= DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL=DB_ACTV_HIC;
    EPwm1Regs.DBFED=0;
    EPwm1Regs.DBRED=0;

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;
    EPwm1Regs.ETSEL.bit.INTEN = 1;
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST; // 1 TBPRD triggers 1 interrupt

    // EPWM 2
    EPwm2Regs.TBPRD = Reg_TBPRD - 1 ;
    EPwm2Regs.TBPHS.half.TBPHS = 0;
    EPwm2Regs.TBCTR = 0;

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN ;
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE ;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_UP ;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1 ;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1 ;
    EPwm2Regs.TBCTL.bit.PRDLD = CC_SHADOW ;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN ;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD ; // CC_CTR_ZERO_PRD ; // Load on Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD ;

    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // Set PWM1A on event A, up count
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;           // Clear PWM1A on event A, down count
    EPwm2Regs.AQCTLA.bit.PRD = AQ_NO_ACTION ;
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION ;
    EPwm2Regs.AQCTLA.bit.CBU = AQ_SET ;
    EPwm2Regs.AQCTLA.bit.CBD = AQ_CLEAR ;

    EPwm2Regs.DBCTL.bit.OUT_MODE= DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL= DB_ACTV_HIC;
    EPwm2Regs.DBFED=0;
    EPwm2Regs.DBRED=0;

    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO ; // when TBCTR=0x0000, the interrupt triggers ;
    EPwm2Regs.ETSEL.bit.INTEN = 1;
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST ; // 1 TBPRD triggers 1 interrupt

    // Set TBCLKSYNC after setup TBCLKSYNC
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=1;
    EDIS;

    EALLOW;
    PieVectTable.EPWM1_INT= &epwm1_timer_isr;
    PieVectTable.EPWM2_INT= &epwm2_timer_isr;
    EDIS;
}
#endif

#endif

