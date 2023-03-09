pos1t = init1 + pos1;
pos2t = init2 + pos2;
%position of optical flow sensor 1 in frame T is initialized position +
%optical flow recorded data.
width = pos1t-pos2t;
%vector between flow sensors
alpha = pi/2-atan(width(2)/width(1));
%angle between flow sensors
R = inv([cos(alpha), sin(alpha); -sin(alpha), cos(alpha)]);
%rotation matrix for the router's frame

orient=R*[0;1];
%if orientation =/= prior movement, reverse

posave = (pos1+pos2)./2;
posrout = pos1t + R*[0.1;0.1];
%PLACEHOLDER. This data will be filled by the position of the gantry.

lin1=[pos1(1) pos1(2) vel1(2)/vel1(1) pos1(2)-vel1(2)/vel1(1)*pos1(1)];
lin2=[pos2(1) pos2(2) vel2(2)/vel2(1) pos2(2)-vel2(2)/vel2(1)*pos2(1)];
lin3=[-vel1(1)/vel1(2) (-vel1(1)/vel1(2))*-1*pos1(1)+pos1(2)];
lin4=[-vel2(1)/vel2(2) (-vel2(1)/vel2(2))*-1*pos2(1)+pos2(2)];

x=(lin4(2)-lin3(2))/(lin3(1)-lin4(1));
y=lin3(1)*x+lin3(2);
ic=[x;y];

om=(norm(vel1))/norm(ic-pos1);
velrout=norm(ic-posrout)*om;
%IC calculations

index=find(min((posrout(1)-x).^2+(posrout(2)-y).^2)==(posrout(1)-x).^2+(posrout(2)-y).^2);
target=index+1;
velgant=velrout/((posrout(1)-g(1,target))/(posrout(2)-g(2,target)));