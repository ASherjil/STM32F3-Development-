clear;clearvars;clc

ARR = 3999
time = 1/100 ;% timer or freq required
dutycycle = 75; % duty cycle in percentage

%----------------------------------------------
syms PSC CCR
sysfreq = 8*10^6;% clock freq of the processor
eqn = ((PSC+1)/(sysfreq))*(ARR+1) == time;
PSC=double(solve(eqn,PSC))
eqn1 = (CCR/(ARR+1))*100 == dutycycle;
CCR = double(solve(eqn1,CCR))
resolution = log(ARR+1)/log(2)
