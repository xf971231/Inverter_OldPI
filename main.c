/** Inverter main program
 *  From Lab209 Prof. YongQiang Ye
 *  by Donghao@NUAA
 *  2017.12.15
 */

#include "main.h"

float rc_output = 0.0 ;
float error_rc_input ;
float error_pi_input ;
float phase = 0.0f ;
float pi_output = 0.0f ;
float current_reference_value = 0.5f;

float debug_scope_1[400] ;
Uint16 debug_scope_count = 0 ;
Uint16  realy_count = 0 ;
Uint16 cnt_current_zero = 0 ;
Uint16 flag_zero = 0 ;
Uint16 flag = 0 ;

float control_modulation = 0.0f ;

pll_sogi_s s_pll_sogi_gird ;
pidStruct pid_ig ;

crc_struct crc ;
crc_struct * p_crc = & crc ;

filter_s low_pass_filter ;
filter_s * p_low_pass_filter = & low_pass_filter ;

float test ;
enum InverterOutputState g_inv_state = InitialState ;

void Delay(void){
    unsigned int i, j = 0 ;
    for(i = 0; i < 65535 ; i ++){
        for(j = 0 ; j < 100; j ++ ){
            asm(" RPT #1 || NOP");
        }
    }
}

/***********************************************************************************************************************
                                                         * main
 **********************************************************************************************************************/
int main(void)
{
/***************************************************************************************************
                                    * System Initialization
 **************************************************************************************************/
    InitSysCtrl();
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    Delay();
#if USE_PWM     //  Initialize Gpio for PWM

    PWM_Init(FREQUENCY_SWITCHING) ;
    // PWM_Init(9890) ;
    // PWM_Init(10110) ;
    // Setup for Interrupts.
    IER|=M_INT3;
    PieCtrlRegs.PIEIER3.bit.INTx1=1;
    PieCtrlRegs.PIEIER3.bit.INTx2=1;
    EINT;
    ERTM;
#endif

    RelayGpioInit();
    KeyGpioInit();

    AD7606_Init();
    InitXintf();    // InitXintf16Gpio() is called inside.

    DAC8554Init();

/***************************************************************************************************
                                  * Controller and PLL Initialization
 **************************************************************************************************/


    control_modulation = 0.0 ;
    g_inv_state = InitialState ;
    //TS_PLL_TWENTY_KHZ
    init_pll_sogi(&s_pll_sogi_gird, KP_PLL, KI_PLL, K_GAIN_SOGI_PLL, TS_TWENTY_KHZ) ; // InitPLL();

    filter_init(p_low_pass_filter, num_filter, den_filter, order_filter ) ;

    init_crc(p_crc, p_low_pass_filter, RC_Q_COEFF, RC_K_RC, RC_LEAD_STEPS) ;

    Init_pidStruct(&pid_ig, C_CTRL_KP, C_CTRL_KI, TS_TWENTY_KHZ ) ;

/***************************************************************************************************
                                             * Main Loop
 **************************************************************************************************/
    for(;;)
    {
        if(flag_timer2_updated)
        {
            SCOPE_PU ;
            flag_timer2_updated = 0 ;
            debug_scope_count ++ ;
            debug_scope_count %= 400 ;
            debug_scope_1[debug_scope_count] =  MeasureBuf[CH_GRID_CURRENT] ;

            if( ErrorDetected() ){
                Shutdown_PWM_RELAY();
                g_inv_state = ErrorEncountered ;
            }else{
                // Do nothing or pull down the IO for the scope to see
                // SCOPE_PD ;
            }

            if(g_inv_state == ErrorEncountered ){
                //SCOPE_PU ;
                Shutdown_PWM_RELAY() ;
            }else{
                //SCOPE_PD ;
            }

/*********************Data Storage************************/
// TODO : store data to external RAM

/*************PLL and Phase angle Generation**************/
            phase = calc_pll_sogi(&s_pll_sogi_gird, MeasureBuf[CH_AC_VOLTAGE] ) ; // 2.2us ;

            if(g_inv_state == CurrentControlled){
                if(phase < 0.1){
                    flag = 1;
                }
                if(flag){
                    cnt_current_zero++;
                }
                if(cnt_current_zero >= 200){
                    cnt_current_zero = 0;
                    flag_zero = 1;
                }
                else{
                    cnt_current_zero = cnt_current_zero;
                    flag_zero = 0;
                }
            }

            if(g_inv_state == CurrentControlled){
                if(flag_zero){
                    flag_zero = 0;
                    current_reference_value = current_reference_value + 0.5;
                    if(current_reference_value >= CURRENT_REF)
                        current_reference_value = CURRENT_REF;
                }
                else{
                    current_reference_value = current_reference_value;
                }

            }

            pid_ig.reference = current_reference_value * sinf(phase) ;

/*****************Control Algorithm***********************/
            // close loop enabled
            if(g_inv_state == CurrentControlled){
                error_rc_input = pid_ig.reference - MeasureBuf[CH_GRID_CURRENT] ;
                // SCOPE_PU ;
                // Real repetitive controller
                // rc_output = calc_crc(p_crc, error_rc_input);
                rc_output = 0.0 ;

                // Simulate repetitive controller
                //rc_output = calc_wdvrc(p_wdvrc, phase, 0.001 * sinf(phase), 1);
                // SCOPE_PD ;
                //debug_scope_1[debug_scope_count] = rc_output ;
                // rc_output = 0.0 ;
                error_pi_input = error_rc_input + rc_output ;
                pi_output = Calc_pidStruct(&pid_ig, error_pi_input) ;

                // Read time damping is used to reduce delay ;
                control_modulation = pi_output // PI controller
                        + MeasureBuf[CH_AC_VOLTAGE] * K_FEEDFORWARD; // PCC voltage feedforward
            }else if(g_inv_state != ErrorEncountered){
                //calc_wdvrc(p_wdvrc, phase, 0, 0) ;
            }
#if 0
            // KEY
            if(KEY_CLOSELOOP_CONTROL_ENABLED && (g_inv_state != ErrorEncountered )){
                // Ensure connecting to the grid when the grid voltage is low ;
                if( fabs(phase - 1.02) < 0.03f){
                    g_inv_state = CurrentControlledPreparation ;
                }
            }

            if(KEY_CLOSELOOP_CONTROL_DISABLED){
                g_inv_state = ErrorEncountered ;
                PWM_DIS ;
                RELAY_1_OPEN ;
                RELAY_2_OPEN ;
            }

            if(g_inv_state == CurrentControlledPreparation){
                PWM_EN ;
                RELAY_1_CLOSEUP ;
                RELAY_2_CLOSEUP ;

                // near the zero crossing point
                if( phase < 0.1){
                    g_inv_state = CurrentControlled ;
                }
                else if( fabs(phase - 3.1415926) < 0.1 ){
                    g_inv_state = CurrentControlled ;
                }
                // when the current is larger than 0.1A
                else if( fabs(MeasureBuf[CH_GRID_CURRENT]) > 0.15){
                    g_inv_state = CurrentControlled ;
                }
            }
#endif

#if 1
            if(KEY_CLOSELOOP_CONTROL_ENABLED && (g_inv_state != ErrorEncountered )){
                RELAY_1_CLOSEUP ;
                RELAY_2_CLOSEUP ;
                // Ensure connecting to the grid when the grid voltage is low ;
                if(phase < 0.1f){
                    g_inv_state = ControlRelay ;
                    realy_count = 0;
                }
            }

            if(g_inv_state == ControlRelay){
               realy_count++;
               if(realy_count >= 600){
                   g_inv_state = ControlPWMOE;
                   realy_count = 0;
               }
               else
                  g_inv_state = ControlRelay;
            }

            //关闭PWMOE和继电器
            if(KEY_CLOSELOOP_CONTROL_DISABLED){
                g_inv_state = ErrorEncountered ;
                PWM_DIS ;
                RELAY_1_OPEN ;
                RELAY_2_OPEN ;
            }

            //PWM使能信号
            if(g_inv_state == ControlPWMOE){
                PWM_EN ;
                // near the zero crossing point
                if( phase < 0.1){
                    g_inv_state = CurrentControlled ;
                }
                else if( fabs(phase - 3.1415926) < 0.1 ){
                    g_inv_state = CurrentControlled ;
                }
                // when the current is larger than 0.1A
                else if( fabs(MeasureBuf[CH_GRID_CURRENT]) > 0.15){
                    g_inv_state = CurrentControlled ;
                }
            }

#endif
/***************End Control Algorithm*********************/

#if USE_DAC

#if !USE_DAC_SPI
//            if(debug_scope_1[debug_scope_count] > 1.0 ){
//                debug_scope_1[debug_scope_count] = 1.0 ;
//            }else if(debug_scope_1[debug_scope_count] < -1.0){
//                debug_scope_1[debug_scope_count] = -1.0 ;
//            }
            if(g_inv_state != ErrorEncountered){
                switch(g_inv_state){
                case CurrentControlled:
                    StoreVoltage(0, 2.5 + 0.001 * rc_output , ADDR_DAC8554, 1);
                    //StoreVoltage(0, 2.5 + 0.2 * pid_ig.reference , ADDR_DAC8554, 1);
                    break ;
                default :
                    StoreVoltage(0, 2.5 + 0.2 * pid_ig.reference , ADDR_DAC8554, 1);
                }
                //StoreVoltage(0, 2 + 0.5 * rc_output , ADDR_DAC8554, 1);
            // StoreVoltage(1,  2 + 2 * debug_scope_1[debug_scope_count] , ADDR_DAC8554, 1);
            }
#else
            // Approximately 2.6us
            DAC8554SpiSetVoltage(MeasureBuf[CH_DC_BUS] * 0.01 , CH_DC_BUS, 0);
            DAC8554SpiSetVoltage(MeasureBuf[CH_AC_VOLTAGE] * 0.005 + 2.5 ,CH_AC_VOLTAGE,0);
            DAC8554SpiSetVoltage(MeasureBuf[CH_GRID_CURRENT] * 0.1 + 2.5 ,CH_GRID_CURRENT,0);
            DAC8554SpiSetVoltage(MeasureBuf[CH_CAP_CURRENT] * 0.1 + 2.5 ,CH_CAP_CURRENT,1);

#endif

#endif
            SCOPE_PD ;
            }
    }
}
