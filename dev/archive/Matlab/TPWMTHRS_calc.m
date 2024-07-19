% Threshold velocity calculation
clear; clc;

%voltage = 20;

f_clk = 12e6;     % Hz
max_usteps = 256;

usteps = 4;
full_steps = 200;
rod_dia = 8;         % mm
v_thres = 1;         % mm/s

spmm = full_steps*usteps / rod_dia;       % steps per mm

TPWMTHRS = f_clk * usteps / (max_usteps * v_thres * spmm);