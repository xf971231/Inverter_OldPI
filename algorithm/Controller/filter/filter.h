#ifndef ALGORITHM_CONTROLLER_FILTER_H_
#define ALGORITHM_CONTROLLER_FILTER_H_

#define MAX_NUMBER_FILTER_COEFFS 4
#define MAX_FILTER_ORDER (MAX_NUMBER_FILTER_COEFFS-1)

typedef struct filter_struct{
    float num_coeff[MAX_NUMBER_FILTER_COEFFS] ;
    float den_coeff[MAX_NUMBER_FILTER_COEFFS] ;
    float x_buffer[MAX_FILTER_ORDER];
    float y_buffer[MAX_FILTER_ORDER];
    int filter_order ;
    int number_filter_coeffs ;
} filter_s ;

float filter_calc(filter_s* fs, float input);
void filter_init(filter_s * fs, const float * num_c, const float * den_c, const int filter_order);

#endif /* ALGORITHM_CONTROLLER_FILTER_H_ */
