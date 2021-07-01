#include "crc.h"

#if SIMULINK_DEBUG 
crc_struct crc_s ;
crc_struct * p_crc_s = & crc_s;
filter_s f_s ;
filter_s * p_f_s = &f_s ;

const double num_filter[3] = {0.019790115127288,   0.039580230254577,   0.019790115127288};
const double den_filter[3] = {1.000000000000000,  -1.564546081037408,   0.643706541546561};
const int order_filter = 2 ;



void init_crc_simulink(){
    int i ; 
    for(i = 0 ; i < STEPS_AT_BASE_FREQ; i ++){
        p_crc_s->q_output_buffer[i] = 0.0 ;
        p_crc_s->delay_prev_buffer[i] = 0.0 ;
    }
    p_crc_s->lead_steps = steps_lead ;

    p_crc_s->krc = k_rc ;
    p_crc_s->pfs = p_f_s ;

    p_crc_s->q_main = q_main ;
    p_crc_s->q_lr = (1 - p_crc_s->q_main) * 0.5 ;

    p_crc_s->q_output_index = 0 ;
    p_crc_s->delay_prev_index = 0 ;
}

double calc_crc_simulink(double error){
    int index_delay_selected ;
    p_crc_s->delay_prev_buffer[p_crc_s->delay_prev_index] = error +
    p_crc_s->q_output_buffer[index_move(p_crc_s->q_output_index, - p_crc_s->lead_steps)] ;

    // update buffer of q filtered buffer:
    index_delay_selected = index_move(p_crc_s->delay_prev_index, -steps_delay + p_crc_s->lead_steps ) ;

    ret_crc = p_crc_s->delay_prev_buffer[index_delay_selected] * p_crc_s->q_main +
        p_crc_s->delay_prev_buffer[index_move(index_delay_selected, 1)] * p_crc_s->q_lr +
        p_crc_s->delay_prev_buffer[index_move(index_delay_selected, -1)] * p_crc_s->q_lr ;

    p_crc_s->q_output_buffer[p_crc_s->q_output_index] = ret_crc ;

    ret_crc = filter_calc(p_crc_s->pfs, ret_crc) ; // Lowpass filter

    // Update index
    p_crc_s->q_output_index = index_move(p_crc_s->q_output_index, 1);
    p_crc_s->delay_prev_index = index_move(p_crc_s->delay_prev_index, 1);

    return ret_crc * p_crc_s->krc ;
}

#else

void init_crc(crc_struct * p_crc_s, filter_s * p_f_s, float q_main, float k_rc, int steps_lead){
    int i ; 
    for(i = 0 ; i < STEPS_AT_BASE_FREQ; i ++){
        p_crc_s->q_output_buffer[i] = 0.0 ;
        p_crc_s->delay_prev_buffer[i] = 0.0 ;
    }
    p_crc_s->lead_steps = steps_lead ;

    p_crc_s->krc = k_rc ;
    p_crc_s->pfs = p_f_s ;

    p_crc_s->q_main = q_main ;
    p_crc_s->q_lr = (1 - p_crc_s->q_main) * 0.5 ;

    p_crc_s->q_output_index = 0 ;
    p_crc_s->delay_prev_index = 0 ;
}

float calc_crc(crc_struct * p_crc_s, float error){
    int index_delay_selected ;
    float ret_crc ;
    p_crc_s->delay_prev_buffer[p_crc_s->delay_prev_index] = error +
    p_crc_s->q_output_buffer[index_move(p_crc_s->q_output_index, - p_crc_s->lead_steps)] ;

    // update buffer of q filtered buffer:
    index_delay_selected = index_move(p_crc_s->delay_prev_index, -STEPS_AT_BASE_FREQ + p_crc_s->lead_steps ) ;

    ret_crc = p_crc_s->delay_prev_buffer[index_delay_selected] * p_crc_s->q_main +
        p_crc_s->delay_prev_buffer[index_move(index_delay_selected, 1)] * p_crc_s->q_lr +
        p_crc_s->delay_prev_buffer[index_move(index_delay_selected, -1)] * p_crc_s->q_lr ;

    p_crc_s->q_output_buffer[p_crc_s->q_output_index] = ret_crc ;

    ret_crc = filter_calc(p_crc_s->pfs, ret_crc) ; // Lowpass filter

    // Update index
    p_crc_s->q_output_index = index_move(p_crc_s->q_output_index, 1);
    p_crc_s->delay_prev_index = index_move(p_crc_s->delay_prev_index, 1);

    return ret_crc * p_crc_s->krc ;
}

#endif


int index_move(int cur_index, int steps_to_move){
    int ret_index = cur_index + steps_to_move ;
    if(ret_index >= STEPS_AT_BASE_FREQ){
        ret_index -= STEPS_AT_BASE_FREQ ;
    }
    else if( ret_index < 0 ){
        ret_index += STEPS_AT_BASE_FREQ ;
    }
    return ret_index ;
}
