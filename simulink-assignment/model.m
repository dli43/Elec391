clear; clc;

% Assign variables
M = 1;
L = 5e-2;
I = M*L^2;
g = 9.81;
s = tf('s');

% Plant transfer function
num = [1];
denom = [I 0 -M*g*L];
Ps = tf(num,denom);

% System transfer function with k control
k = 12;
CF = 100;
Dp = CF/(s*(s+CF));
Hs = feedback(k*Ps,1);
pole(Hs)

