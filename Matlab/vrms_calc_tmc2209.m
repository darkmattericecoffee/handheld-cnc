%% Vref calculator (for TMC2209)
I = 0.9;               % current rating of motor
r_sense = 110;       % mOhm

Irms = I/sqrt(2);

Vref = 2.5*sqrt(2)*(r_sense + 20)*Irms/325;