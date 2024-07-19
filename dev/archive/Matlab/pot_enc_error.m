% Error propogation script for string pot/encoder sensing method
% This uses a string potentiometer and a rotary encoder
clear; clc;

% Declaring variables
%syms r th

% Error values
pot_error = 0.001;     % error in m
rot_error = 0.01;       % error in rad

% Sample data
%t = 0:200;        % s (doesn't matter that their not equal distance apart)
x = 0:0.01:1;
y = sin(x);
th = [0, atan(y(2:end)./x(2:end))];
r = sqrt(x.^2 + y.^2);

% Error propogation
Ux = sqrt((pot_error*cos(th)).^2 + (rot_error*r.*sin(th)).^2);
Uy = sqrt((pot_error*sin(th)).^2 + (rot_error*r.*cos(th)).^2);

%Uz = Ux(:)*Uy(:).';    % meaningless
disp(max(Ux));

% Plotting
plot(x,y)
hold on
rectangle('Position',[0,0,1,1])
for i = 1:length(x)
   pos = [x(i) - Ux(i) / 2, y(i) - Uy(i) / 2];
   rectangle('Position', [pos,Ux(i),Uy(i)])
end
hold off