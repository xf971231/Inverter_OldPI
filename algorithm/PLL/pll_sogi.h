#ifndef PLL_SOGI_H
#define PLL_SOGI_H 

#include "math.h"

#define DOUBLE_PI_PLL_SOGI 6.283185307179586

typedef struct pll_sogi_s{
    float sogi_forward_integrator_output ;
    float sogi_backward_integrator_output ;
    float phase_output ;
    float loop_filter_kp ;
    float loop_filter_ki ;
    float angle_freq ;
    float update_period ;
    float center_angle_freq ;
    float loop_filter_sum ;
    float k_gain_sogi ;
} pll_sogi_s ;

void init_pll_sogi(pll_sogi_s * p, float kp, float ki, float k_gain_sogi, float update_period);
float calc_pll_sogi(pll_sogi_s * p, float pcc_voltage) ;

#endif
