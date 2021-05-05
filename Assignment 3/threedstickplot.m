% x = []; 
% y = []; 
% z = []; 
% all in metres

square_width = 3.9/100;
edge_buffer = 2.1/100;
origin_offset = 3/100;
grab_height = 3.5/100;
avoid_height = 15/100;

d1 = 0.05;      % shoulder
a2 = 0.60;      % bicep
a3 = 0.50;      % forearm
dE = 0.15;      % hand length

th1 = 0; 
th2 = 0;
th3 = 0;


% Forward Kinematics the links
L1_start = [0;0;d1]
L1_end = [a2*cosd(th1)*cosd(th2); -a2*cosd(th2)*sind(th1); a2*sind(th2) + d1];

L2_start = L1_end
L2_end = [a2*cosd(th1)*cosd(th2) + a3*cosd(th1)*cosd(th2+th3); -a2*cosd(th2)*sind(th1) + a3*sind(th1)*cosd(th2+th3); a2*sind(th2) + a3*sind(th2+th3) + d1];

EE_start = L2_end;
EE_end = L2_end + [0; 0; -dE];


L1 = [L1_start, L1_end];
L2 = [L2_start, L2_end];
EE = [EE_start, EE_end];

% Intitial XZ plot

% Plot stem
plot([0 0], [0 L1_start(3)]);
hold on;
% Plot Link1
plot([L1_start(1) L1_end(1)], [L1_start(3) L1_end(3)]);
hold on;
% Plot Link2
plot([L2_start(1) L2_end(1)], [L2_start(3) L2_end(3)]);
hold on;
% Plot Link3
plot([EE_start(1) EE_end(1)], [EE_start(3) EE_end(3)]);
hold on;


axis padded;
grid on


%{ 
    change variable definition, 
    daisy chained -->  address serial, 
    but servos don't know their own address --> upload to servo, 
    connect and launch feetech debug software (tells servo what their ID name is) --> 
    sudddenly it works (only velocity inputs in HEX)

    can't write position but can read,
    servos are continuous (360)
    encoder is 0 - 270 deg
    anything beyond 270 = 0 (need to spin back)
    Voltage: 0 -> 1023

    library fixed (lms)
%}

% th4 and th5 redund
function [th1,th2,th3] = invKin(x, y, z, d1, a2, a3)
    th1 = atan2d(-y/x);
    H = sqrt(x^2+y^2+(z-d1)^2);
    
    th2 = atan2d((z-d1)/sqrt(x^2+y^2))+acosd((a2^2+H^2-a3^2)/(2*a2*H));
    th3 = acosd((a2^2+a3^3-H^2)/(2*a2*a3))-180;
    
    % th4 = -th2 - th3;
    % th5 = th1;
end

