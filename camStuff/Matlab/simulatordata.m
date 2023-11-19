clear all;
clc;
close all;

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