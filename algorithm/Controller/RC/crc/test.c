#include "crc.h"
#include "stdio.h"
#include "stdlib.h" 

crc_struct crc_s ;
crc_struct * p_crc_s = & crc_s;
filter_s f_s ;
filter_s * p_f_s = &f_s ;

float num_filter[3] = {0.019790115127288,   0.039580230254577,   0.019790115127288};
float den_filter[3] = {1.000000000000000,  -1.564546081037408,   0.643706541546561};
int order_filter = 2 ;


int main(void){
    float error, phase, steps_to_see;
    FILE * fp = fopen("test_error_phase.txt", "r") ;
    if(fp == NULL){
        printf("Error openning file\r\n");
        exit(0) ;
    }
    FILE * fp_out = fopen("text_output.txt", "w") ;
    if(fp_out == NULL){ 
        printf("Error creating file\r\n") ;
        exit(0) ;
    }

    printf("Initialization...\r\n");
    filter_init(p_f_s, num_filter, den_filter, order_filter) ;
    init_crc(p_crc_s, p_f_s, 0.9, 1.0, 5) ;
    printf("Initialization completed!\r\n");

    for( int i = 0 ; i < 1e6 ; i ++){
        fscanf(fp, "%f\r\n", &error) ;
        steps_to_see = calc_crc( p_crc_s, error) ;
        fprintf(fp_out, "%f\r\n", steps_to_see) ;
    }

    fclose(fp) ;
    fclose(fp_out) ;
    return 0 ;
}