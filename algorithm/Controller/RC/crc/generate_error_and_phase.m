close all ;
clear ;
clc ;

freq = 2e4/400  ;
samples_f = 2e4 / freq ; 
samples = int32(1e4 / freq);

t = 5e-5:5e-5:50 ; 
ph = 2 * pi * freq * t ;

error = sin(ph) ;
figure ;
plot(t, error) ;

dlmwrite('test_error_phase.txt', error') ;

 