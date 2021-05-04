function [th1,th2,th3,th4,th5] = invKin(x,y,z)
    th1 = atan2d(-y/x);
    th5 = th1;
    %th2
    %th3
    th4 = = -th2 - th3;
end

% x = []
%y = []
%z = []
% a2
% a3

% Forward Kinematics the links
link1start = [0;0;d1];
link1end = [a2*cos(th1)*cos(th2);-a2*cos(th2)*sin(th1);a2*sin(th2)];
link2start = link1end;
link2end = [a2*cos(th1)*cos(th2)+a3*cos(th1)*cos(th2+th3);-a2*cos(th2)*sin(th1)+a3*sin(th1)*cos(th2+th3);a2*sin(th2)+a3*sin(th2+th3)];
EEstart = link2end;
EEend = link2end - [0;0;-dE];

l1 = [link1start,link1end];
l2 = [link2start,link2end];
EE = [EEstart,EEend];
plot(l1(1,:),l1(2,:),l1(3,:))
hold on
plot(l2(1,:),l2(2,:),l2(3,:))
hold on
plot(EE(1,:),EE(2,:),EE(3,:))
hold on

