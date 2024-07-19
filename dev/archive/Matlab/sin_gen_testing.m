clear; clc;
% Sine generator testing
amp = 20;
period = 200;
y = linspace(0,200,1000);
x = amp*sin((2*pi/period)*y);

plot(x,y)
hold on
xline(50)
xline(-50)
hold off
axis equal