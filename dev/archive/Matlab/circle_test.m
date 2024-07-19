clear; clc;
num_points = 1000;
diameter = 100;
radius = diameter / 2;
angle_step = 2 * pi / num_points;

x = zeros(1, num_points);
y = zeros(1, num_points);

for i = 1:num_points
  angle = angle_step * (i - 1);
  x(i) = -radius + radius * cos(angle);
  y(i) = radius * sin(angle);
end

plot(x,y)
axis equal