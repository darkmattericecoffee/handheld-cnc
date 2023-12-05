% Crossing paths
clear; clc;

rout = [0,0];
theta = pi/6;
g = [1,4];

% Strat 1: Using angle
alpha = atan((g(2) - rout(2))/(g(1) - rout(1))) - theta;

% Strat 2: Using sine of angle
%disp(sin(alpha))
val = sin(alpha);

% Strat 3: Using signed distance
A = tan(theta);
B = -1;
C = rout(2) - A*rout(1);

dist = (A*g(1) + B*g(2) + C)/sqrt(A^2 + B^2);