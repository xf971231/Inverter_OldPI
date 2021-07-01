#ifndef WDVRC_H
#define WDVRC_H

#include "../../filter/filter.h"
#include "math.h"

#define USE_TEN_KHZ 0
#define USE_TWENTY_KHZ 1

#if USE_TEN_KHZ
#define STEPS_AT_BASE_FREQ 200
#endif
#if USE_TWENTY_KHZ
#define STEPS_AT_BASE_FREQ 400
#endif


#define DOUBLE_PI_DVRC 6.28318530718
#define PI_DVRC 3.14159265359

#define SIMULINK_DEBUG 0

#if SIMULINK_DEBUG
typedef struct crc_structure{
    double phase_buffer[MAX_STEPS_FOR_BUFFER] ;
    int phase_index ;
    double q_output_buffer[MAX_STEPS_FOR_BUFFER] ;
    int q_output_index ;
    double delay_prev_buffer[MAX_STEPS_FOR_BUFFER] ;
    int delay_prev_index ;

    int lead_steps ;
    filter_s * pfs ;
    double q_main ;
    double q_lr ;

    double krc ;
    double weight_dm ; // delay 1 more step ; e.g. 202
    double weight_dl ; // delay 1 less step ; e.g. 201

    int error_flag  ;

}crc_struct ;

void init_wcrc_simulink() ;
double calc_wcrc_simulink(double phase, double error, int flag_start_calc);
int find_varying_delay(crc_struct * ps, double cur_phase);
double compare_phase_angle(double cur_phase, double prev_phase) ;
int index_move(int cur_index, int steps_to_move);

#else

typedef struct crc_structure{
    float q_output_buffer[STEPS_AT_BASE_FREQ] ;
    int q_output_index ;
    float delay_prev_buffer[STEPS_AT_BASE_FREQ] ;
    int delay_prev_index ;

    int lead_steps ;
    filter_s * pfs ;
    float q_main ;
    float q_lr ;

    float krc ;
}crc_struct ;

void init_crc(crc_struct * p_crc_s, filter_s * p_f_s, float q_main,  float k_rc, int steps_lead);
float calc_crc(crc_struct * p_crc_s, float error);
int index_move(int cur_index, int steps_to_move);

#endif

#endif
