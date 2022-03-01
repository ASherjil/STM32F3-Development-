clear;clearvars;clc

PSC = 3999
time = 100^-1;% timer or freq required
dutycycle = 25; % duty cycle in percentage

%----------------------------------------------
syms ARR CCR
sysfreq = 8*10^6;% clock freq of the processor
eqn = ((PSC+1)/(sysfreq))*(ARR+1) == time;
ARR=double(solve(eqn,ARR))
eqn1 = (CCR/(ARR+1))*100 == dutycycle;
CCR = double(solve(eqn1,CCR))
