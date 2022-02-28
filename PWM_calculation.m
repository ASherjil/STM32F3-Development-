clear;clearvars;clc


Psc = 7999

syms arr duty
eqn = ((Psc+1)/(8*10^6))*(arr+1) == (100^-1);
arr=double(solve(eqn,arr))

% duty cycle in percentage
dutycycle = 10; 
eqn1 = (duty/(arr+1))*100 == dutycycle;
duty = double(solve(eqn1,duty))
