/*************************************************************
Author: Hao Dong
Date: 2018.06.23
Email:  hao.dong.nanjing@gmail.com
*************************************************************/

#ifndef PID_H
#define PID_H
#define PID_KD_ENABLED 0

typedef struct {
    float kp ;
    float ki ;
    float sum_ki ;
    float isum_low ;
    float isum_high ;
    float output ;
    float reference ;
} pidStruct;

void Init_pidStruct(pidStruct * s, float, float, float) ;
float Calc_pidStruct(pidStruct * s, float error);

#endif
