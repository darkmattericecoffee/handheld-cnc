clear; clc;

% Define points
amp = 20;
period = 200;
y = linspace(0,200,1000);
x = -amp*sin((2*pi/period)*y);
C = [x(1),y(1)];

A = [x(2),y(2)];

beta = atan((A(2) - C(2))/(A(1) - C(1)));

%theta = atan(C(2)/C(1));
theta = 0;

alpha = (pi/2) - beta - theta


%plot([C(1) A(1)],[C(2) A(2)])
scale = 100;
plot(x,y)
hold on
plot(scale*x(1:2),scale*y(1:2))
hold off
axis equal