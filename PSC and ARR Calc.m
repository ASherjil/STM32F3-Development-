clear;clearvars;clc

ARR = 3999
time = 1/100 ;% timer or freq required
dutyCycles=[0 10 25 50 75 90 100];



%---------------------------------------------
resolution = log(ARR+1)/log(2)
syms PSC
sysfreq = 8*10^6;% clock freq of the processor
eqn = ((PSC+1)/(sysfreq))*(ARR+1) == time;
PSC=double(solve(eqn,PSC))

formatSpec = 'Duty cycle %d= CCR of %f';

for i = 1:length(dutyCycles)
syms CCR
duty = dutyCycles(i);
eqn1 = (CCR/(ARR+1))*100 == duty;
CCR = double((solve(eqn1,CCR)));
str= sprintf(formatSpec,duty,CCR)
end
