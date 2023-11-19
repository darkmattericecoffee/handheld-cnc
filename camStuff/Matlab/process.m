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
pos1t = pos1(:,c);
pos2t = pos2(:,c);
%importing simulation data. These are just the readings we get out of each
%optical flow sensor.
width = pos1t-pos2t;
%vector between flow sensors
alpha = pi/2-atan(width(2)/width(1));
%angle between flow sensors
R = [cos(alpha), sin(alpha); -sin(alpha), cos(alpha)];
%rotation matrix for the router's frame

orient=R*[1;0];
%orientation vector for the CNC, always points in the orientation of
%movement

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