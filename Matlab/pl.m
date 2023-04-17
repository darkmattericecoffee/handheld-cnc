plot(pos2(1,1:len),pos2(2,1:len),'o')
hold on
plot(pos1(1,1:len),pos1(2,1:len),'o')
plot(g(1,1:17000),g(2,1:17000))
%% 
figure

plot(g(1,:),g(2,:))
hold on
gantry=(pos2+pos1)./2;
plot(gantry(1,1:len),gantry(2,1:len),'x');
plot(posrout(1,:),posrout(2,:))

plot(pos1t(1),pos1t(2),'o')
hold on
plot(pos2t(1),pos2t(2),'o')
plot(posrout(1),posrout(2),'o')