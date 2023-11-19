clear all;
clc;
close all;

%% Simulator Data

len=50000;
scafa=100;

x=linspace(0,50,len);
y=0.3*sin(x*3)+1.5;
g=[x;y];

%init
t=linspace(0,5,len+1);
IC=[linspace(0,5,len+1);linspace(1.5,5,len+1);linspace(0,0,len+1)];
pos1=[linspace(0,5,len+1);linspace(3,5,len+1);linspace(0,0,len+1)];
pos2=[linspace(0,5,len+1);linspace(0,5,len+1);linspace(0,0,len+1)];
om=[linspace(0,0,len+1);linspace(0,0,len+1);linspace(0,0,len+1)];
v1=[linspace(0,0,len+1);linspace(0,0,len+1);linspace(0,0,len+1)];
v2=[linspace(0,0,len+1);linspace(0,0,len+1);linspace(0,0,len+1)];
r1=[linspace(0,0,len+1);linspace(0,0,len+1);linspace(0,0,len+1)];
r2=[linspace(0,0,len+1);linspace(0,0,len+1);linspace(0,0,len+1)];

for c=1:len
    %IC(1,c)=[t(c)+exp(-0.1*t(c))*sin(t(c))];
    %IC(2,c)=[0.1*sin(t(c))+1.5];
    %pos1(1,c)=[t(c)+exp(-0.1*t(c))*0.03*sin(4*pi*t(c))];
    %pos1(2,c)=[0.2*sin(2*t(c))+3];
    IC(:,c)=pos1(:,c)-[2*sin(5*t(c)/scafa);0.1*cos(5*t(c)/scafa)+20;0];
    r1(:,c)=pos1(:,c)-IC(:,c);
    r2(:,c)=pos2(:,c)-IC(:,c);
    
    om(3,c)=-abs(0.0025*sin(100*t(c)/scafa))/scafa;
    
    %v1(:,c)=[0.1*sin(2*t(c))+1;0;0]+cross(om(:,c),r1(:,c));
    %v2(:,c)=[0.1*sin(2*t(c))+1;0;0]+cross(om(:,c),r2(:,c));
    v1(:,c)=cross(om(:,c),r1(:,c));
    v2(:,c)=cross(om(:,c),r2(:,c));
    pos1(:,c+1)=v1(:,c)+pos1(:,c);
    pos2(:,c+1)=v2(:,c)+pos2(:,c);
end

%plots
% plot(t,IC(1,:))
% plot(IC(1,:),IC(2,:))
% plot(pos1(1,:),pos1(2,:))
% plot(pos2(1,:),pos2(2,:))

%% Processing
velrout=[linspace(0,100,len);linspace(0,100,len);linspace(0,100,len)];
velgant=[linspace(0,100,len)];
posgant=1.5;
%initialized the router position as directly in the middle of the gantry.
ic=[linspace(0,0,len);linspace(0,0,len)];
ohm=[linspace(0,0,len);linspace(0,0,len);linspace(0,0,len)];
posrout=[linspace(0,0,len);linspace(0,0,len);linspace(0,0,len)];
for c=1:len

vel1=v1(:,c);
vel2=v2(:,c);
pos1t = pos1(:,c);      % readings from each optical flow sensor
pos2t = pos2(:,c);   
width = pos1t-pos2t;                   %vector between flow sensors
alpha = pi/2-atan(width(2)/width(1));  %angle between flow sensors
R = [cos(alpha), sin(alpha); -sin(alpha), cos(alpha)];
%rotation matrix for the router's frame

orient=R*[1;0];
%orientation vector for the CNC, always points in the orientation of
%movement (technically this is untrue. This is just the y-axis direction in
%the body frame)

posroutf=[0;posgant];
%position of the router in frame F (the CNC's frame) with origin at the
%position of the 2nd optical flow sensor.
posrout(:,c)=[R*posroutf;0]+pos2t;
%position of the router in frame T with origin at 0,0

velrout(:,c)=(vel1+vel2)./2;
%approximation of the velocity router through an average of velocities.
%Accuracy can be improved through weighted averages, as when the router is
%closer to either flow sensor, the velocity of the router will be closer to
%that.
velpe=orient*dot(velrout([1,2],c),orient);
%motion of the router in the direction of the orientation vector
velpa=velrout([1;2],c)-velpe;
%motion of the router in the perpendicular direction to the orientation
%vector

index=find(min(abs(posrout(1,c)-x))==abs(posrout(1,c)-x));
%finds the closest point on the inputted line plot to the router
target=index+1;
%shoots for the next point on the line
gf=inv(R)*(g(:,target)-posrout(1:2,c));
%finds the vector between the current router position and the targeted
%point on the line in frame F
if gf(1)<0
    gf(1)=abs(gf(1));
end

velgant(c)=norm(velpe)/(gf(1)/gf(2))-norm(velpa);
%calculates necessary gantry velocity in the direction of the parallel
%velocity to achieve the correct angle

posgant=posgant+velgant(c)*1;
%integrator not necessary when gantry values are known.
end