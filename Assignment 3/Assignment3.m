clear all

%% Assignment 3
%% Define Constants
syms Q1 Q2 Q3 
Q4 = -Q3-Q2
Q5 = 0

syms L1 L2 L3 L4 L5
L = [L1 L2 L3 L4 L5]

syms m1 m2 m3 m4 m5
m = [m1 m2 m3 m4 m5]

% Change line 340 to 344 chessPos2Metre() funcntion
board_dimension = 0.30; % 30x30 cm^2
tile = board_dimension/8;
tile_center = tile/2; % Center of tile
offset_x = -0.1; % Robot Offset in x direction
offset_y = 4*tile; % Robot Offset in y direction

% Plot
tiledlayout(3,3);

%% Derive Velocity Relationship
% DH Table
a = [0 0 L2 L3 0 0]
alpha = [0 pi/2 0 0 pi/2 0]
d = [L1 0 0 0 L4 L5]
theta = [Q1 Q2 Q3 Q4 Q5 0]

% All Transformation Matrices
TM = fk(a, alpha, d, theta);

%% Trajectory Generation
syms x_i xd_i x_f xd_f y_i yd_i y_f yd_f z_i zd_i z_f zd_f t_tr % Translation
syms w_i w_f t_rot

% Constants
t_tr = 3;
t_rot = 3;
w_i = 0;
w_f = 0;

% Direct from A to B
% Translation
[x_a0 x_a1 x_a2 x_a3] = cubicpoly(x_i, xd_i, x_f, xd_f, t_tr);
[y_a0 y_a1 y_a2 y_a3] = cubicpoly(y_i, yd_i, y_f, yd_f, t_tr);
[z_a0 z_a1 z_a2 z_a3] = cubicpoly(z_i, zd_i, z_f, zd_f, t_tr);
% Rotation Reference
Rref = [1 0 0; 0 -1 0; 0 0 -1]; % Positive x rotation by pi, align effector downwards
T_effector = eye(4);
sz = size(a);
numframe = sz(2);
for i = [1:numframe]
    T_effector = T_effector*TM{i};
end
R_effector = T_effector(1:3,1:3);

% Direct rotation
[R_direct, theta_direct] = orTrajectory(Rref, R_effector, w_i, w_f, t_rot);

% Using 2 via points, check height constant in function
% height = 0.2; % 20 cm lift
% % Via point 1, directly above chess piece, only z will have movement
% syms x_v1 xd_v1 y_v1 yd_v1 z_v1 zd_v1 w_v1
% xd_i = 0;
% x_v1 = x_i;
% xd_v1 = 0; % Not moving in x
% yd_i = 0;
% y_v1 = y_i;
% yd_v1 = 0; % Not moving in y
% z_v1 = z_i + height;
% zd_v1 = 0; % Stop at correct z position
% w_v1 = 0;
% [v1_x_a0 v1_x_a1 v1_x_a2 v1_x_a3] = cubicpoly(x_i, xd_i, x_v1, xd_v1, t_tr);
% [v1_y_a0 v1_y_a1 v1_y_a2 v1_y_a3] = cubicpoly(y_i, yd_i, y_v1, yd_v1, t_tr);
% [v1_z_a0 v1_z_a1 v1_z_a2 v1_z_a3] = cubicpoly(z_i, zd_i, z_v1, zd_v1, t_tr);
% [R_v1, theta_v1] = orTrajectory(Rref, R_effector, w_i, w_v1, t_rot);
% 
% % Via point 2, directly above destination
% syms x_v2 xd_v2 y_v2 yd_v2 z_v2 zd_v2 w_v2
% x_v2 = x_f;
% xd_v2 = 0; % Stop x movement when reach destination
% y_v2 = y_f;
% yd_v2 = 0; % Stop y movement when reach destination
% z_v2 = z_v1;
% zd_v2 = 0; % Not moving in z
% w_v2 = 0;
% [v2_x_a0 v2_x_a1 v2_x_a2 v2_x_a3] = cubicpoly(x_v1, xd_v1, x_v2, xd_v2, t_tr);
% [v2_y_a0 v2_y_a1 v2_y_a2 v2_y_a3] = cubicpoly(y_v1, yd_v1, y_v2, yd_v2, t_tr);
% [v2_z_a0 v2_z_a1 v2_z_a2 v2_z_a3] = cubicpoly(z_v1, zd_v1, z_v2, zd_v2, t_tr);
% [R_v2, theta_v2] = orTrajectory(Rref, R_effector, w_v1, w_v2, t_rot);
% 
% % Final position
% [f_x_a0 f_x_a1 f_x_a2 f_x_a3] = cubicpoly(x_v2, xd_v2, x_f, xd_f, t_tr);
% [f_y_a0 f_y_a1 f_y_a2 f_y_a3] = cubicpoly(y_v2, yd_v2, y_f, yd_f, t_tr);
% [f_z_a0 f_z_a1 f_z_a2 f_z_a3] = cubicpoly(z_v2, zd_v2, z_f, zd_f, t_tr);
% [R_f, theta_f] = orTrajectory(Rref, R_effector, w_v2, w_f, t_rot);

%% Move Rook from A1 to E1
% Check constants used in chessPos2Metre

% Rook initial position (1,1)
rook_init_pos = [1 1];

% Rook final position (5,1)
rook_final_pos = [8 1];

% 2 Via Points
syms t
[v1, v2, vf, theta_v1, theta_v2, theta_f] = via2point(rook_init_pos, rook_final_pos, Rref, R_effector, t_tr, t_rot);

%% Plot
density = 1000;
t = 0:t_tr/density:t_tr;

% Define Link Length
L1 = 25/100;
L2 = 24/100;
L3 = 25/100;
L4 = 0/100;
L5 = 15/100;
link = subs(L);

% Time component plots
v1_x = v1{1};
v2_x = v2{1};
vf_x = vf{1};
nexttile
hold on
title('x-axis movement')
plot(t,v1_x(t))
plot(t+t_tr,v2_x(t))
plot(t+t_tr+t_tr,vf_x(t))
legend('init to via\_point 1', 'via\_point1 to via\_point2', 'via\_point2 to final')
xlabel('time (s)')
ylabel('effector position (m)')

v1_y = v1{2};
v2_y = v2{2};
vf_y = vf{2};
nexttile
hold on
title('y-axis movement')
plot(t,v1_y(t))
plot(t+t_tr,v2_y(t))
plot(t+t_tr+t_tr,vf_y(t))
legend('init to via\_point 1', 'via\_point1 to via\_point2', 'via\_point2 to final')
xlabel('time (s)')
ylabel('effector position (m)')

v1_z = v1{3};
v2_z = v2{3};
vf_z = vf{3};
nexttile
hold on
title('z-axis movement')
plot(t,v1_z(t))
plot(t+t_tr,v2_z(t))
plot(t+t_tr+t_tr,vf_z(t))
legend('init to via\_point 1', 'via\_point1 to via\_point2', 'via\_point2 to final')
xlabel('time (s)')
ylabel('effector position (m)')

% Spatial Plot
index = 1;
for i = 0:3*t_tr
   if i <= t_tr % First viapoint
       x(index) = v1_x(i);
       y(index) = v1_y(i);
       z(index) = v1_z(i);
   elseif i > t_tr && i <= 2*t_tr % Second viapoint
       x(index) = v2_x(i-t_tr);
       y(index) = v2_y(i-t_tr);
       z(index) = v2_z(i-t_tr);
   else
       x(index) = vf_x(i-2*t_tr);
       y(index) = vf_y(i-2*t_tr);
       z(index) = vf_z(i-2*t_tr);
   end
   index = index+1;
end

% XY Plot
nexttile
hold on
title('XY-Plane (Birds eye view)')
plot(x,y, 'b')
plot([0-offset_x board_dimension-offset_x board_dimension-offset_x 0-offset_x 0-offset_x], [0-offset_y, 0-offset_y, board_dimension-offset_y, board_dimension-offset_y, 0-offset_y], '--k') % Chessboard 30x30
xlim([-0.05 board_dimension-offset_x+0.05])
ylim([-board_dimension/2-0.05 board_dimension/2+0.05])
plot(0,0,'*g') % Base position
text(-0.035,0.015,'Robot Position')
xlabel('X-Axis (m)')
ylabel('Y-Axis (m)')
legend('Effector movement in X-Y', 'Chessboard')

% ZX Plot
nexttile
hold on
title('ZX-Plane (Side view)')
plot([0 0],[0 L1], 'g') % Base position
plot(x,z, 'b')
plot([0-offset_x board_dimension-offset_x], [0 0], 'k') % Chessboard
xlim([-0.05 board_dimension+0.06])
ylim([-0.01 0.05+board_dimension+0.06-0.01])
for i = 1:9
    plot([tile*(i-1)-offset_x tile*(i-1)-offset_x], [0 -0.01], 'k') % Chessboard tile
end
xlabel('Effector X-Position (m)')
ylabel('Effector Z-Position (m)')
legend('Robot Position with Link 1', 'Movement of Effector in Z-X', 'Chessboard')

%% Plot 4
density = 10;
t = 0:t_tr/density:t_tr;
plot4_x = [v1_x(t) v2_x(t) vf_x(t)];
plot4_y = [v1_y(t) v2_y(t) vf_y(t)];
plot4_z = [v1_z(t) v2_z(t) vf_z(t)];

sz = size(plot4_x);
limit = sz(2);
Q1_val = zeros(1,limit);
Q2_val = zeros(1,limit);
Q3_val = zeros(1,limit);
Q4_val = zeros(1,limit);

for i = [1:limit]
   coor = [plot4_x(i) plot4_y(i) plot4_z(i)];
   [Q1_val(i), Q2_val(i), Q3_val(i), Q4_val(i)] = ik(coor, link);
end

% Get variables for Links
TM_Link1 = eye(4);
for i = [1:2]
   TM_Link1 = TM_Link1 * TM{i}; 
end
Link1_xvar = TM_Link1(1, 4);
Link1_yvar = TM_Link1(2, 4);
Link1_zvar = TM_Link1(3, 4);

TM_Link2 = eye(4);
for i = [1:3]
    TM_Link2 = TM_Link2 * TM{i};
end
Link2_xvar = TM_Link2(1, 4);
Link2_yvar = TM_Link2(2, 4);
Link2_zvar = TM_Link2(3, 4);

TM_Link3 = eye(4);
for i = [1:4]
    TM_Link3 = TM_Link3 * TM{i};
end
Link3_xvar = TM_Link3(1, 4);
Link3_yvar = TM_Link3(2, 4);
Link3_zvar = TM_Link3(3, 4);

TM_Link4 = eye(4);
for i = [1:6]
    TM_Link4 = TM_Link4 * TM{i};
end
Link4_xvar = TM_Link4(1, 4);
Link4_yvar = TM_Link4(2, 4);
Link4_zvar = TM_Link4(3, 4);

% Get position of each Link/Frame origin
Link1_x = zeros(1,limit);
Link1_y = zeros(1,limit);
Link1_z = zeros(1,limit);
Link2_x = zeros(1,limit);
Link2_y = zeros(1,limit);
Link2_z = zeros(1,limit);
Link3_x = zeros(1,limit);
Link3_y = zeros(1,limit);
Link3_z = zeros(1,limit);
Link4_x = zeros(1,limit);
Link4_y = zeros(1,limit);
Link4_z = zeros(1,limit);

for i = [1:limit]
    Q1 = deg2rad(Q1_val(i));
    Q2 = deg2rad(Q2_val(i));
    Q3 = deg2rad(Q3_val(i));
    Q4 = deg2rad(Q4_val(i));
    Link1_x(i) = subs(Link1_xvar);
    Link1_y(i) = subs(Link1_yvar);
    Link1_z(i) = subs(Link1_zvar);
    Link2_x(i) = subs(Link2_xvar);
    Link2_y(i) = subs(Link2_yvar);
    Link2_z(i) = subs(Link2_zvar);
    Link3_x(i) = subs(Link3_xvar);
    Link3_y(i) = subs(Link3_yvar);
    Link3_z(i) = subs(Link3_zvar);
    Link4_x(i) = subs(Link4_xvar);
    Link4_y(i) = subs(Link4_yvar);
    Link4_z(i) = subs(Link4_zvar);
end

% Actual plotting
% XY
nexttile
hold on
title('XY-Plane (Birds eye view)')
plot(x,y, 'b')
plot([0-offset_x 0.3-offset_x 0.3-offset_x 0-offset_x 0-offset_x], [0-offset_y, 0-offset_y, 0.3-offset_y, 0.3-offset_y, 0-offset_y], '--k') % Chessboard 30x30
xlim([-0.05 board_dimension-offset_x+0.05])
ylim([-board_dimension/2-0.05 board_dimension/2+0.05])
plot(0,0,'*g') % Base position
text(-0.035,0.015,'Robot Position')
xlabel('X-Axis (m)')
ylabel('Y-Axis (m)')
for i = [1:limit]
   plot([0 Link1_x(i) Link2_x(i) Link3_x(i) Link4_x(i)], [0 Link1_y(i) Link2_y(i) Link3_y(i) Link4_y(i)]) 
end
legend('Effector movement in X-Y', 'Chessboard')

nexttile
hold on
title('ZX-Plane (Side view)')
plot(x, z, 'b')
plot([0-offset_x 0.3-offset_x], [0 0], 'k') % Chessboard
xlim([-0.05 board_dimension-offset_x+0.05])
ylim([-0.01 max(Link2_z)+0.05])
for i = 1:9
    plot([tile*(i-1)-offset_x tile*(i-1)-offset_x], [0 -0.01], 'k') % Chessboard tile
end
for i = [1:limit]
%    plot([0 Link1_x(i) Link4_x(i)], [0 Link1_z(i) Link4_z(i)]) % To check if reachable
%    plot([0 Link1_x(i) Link2_x(i) Link3_x(i) Link4_x(i)], [0 Link1_z(i) Link2_z(i) Link3_z(i) Link4_z(i)])
   plot([0 Link1_x(i)], [0 Link1_z(i)], 'g')
   plot([Link1_x(i) Link2_x(i)], [Link1_z(i) Link2_z(i)], 'y')
   plot([Link2_x(i) Link3_x(i)], [Link2_z(i) Link3_z(i)], 'm')
   plot([Link3_x(i) Link4_x(i)], [Link3_z(i) Link4_z(i)], 'r')
end
xlabel('Effector X-Position (m)')
ylabel('Effector Z-Position (m)')
legend('Movement of Effector in Z-X', 'Chessboard')

%% Functions
%% Forward Kinematics Function
% Input: DH table varaibles
% Output: Transformation Matrix 4x4 with respect to frame 0
function T = fk(a,alpha,d,theta)
sz = size(theta);
n = sz(2);

T = cell(n,1);
T_i = eye(4);

for i = [1:n]
    Dx = [1 0 0 a(i);
          0 1 0 0;
          0 0 1 0;
          0 0 0 1];
    
    Rx = [1    0               0         0;
          0 cos(alpha(i)) -sin(alpha(i)) 0;
          0 sin(alpha(i)) cos(alpha(i))  0;
          0    0               0         1;];
    
    Dz = [1 0 0 0;
          0 1 0 0;
          0 0 1 d(i);
          0 0 0 1];
    
    Rz = [cos(theta(i)) -sin(theta(i)) 0 0;
          sin(theta(i)) cos(theta(i))  0 0;
          0                 0          1 0;
          0                 0          0 1];
    
    T{i} = simplify(Dx * Rx * Dz * Rz);
end

end

%% Inverse Kinematics Function
% Input: Coordinates of end effector [x,y,z] and link length [L1,L2,L3,L4,L5]
% Output: [Q1 Q2 Q3 Q4]
function [Q1 Q2 Q3 Q4] = ik(coor,link)
% Define variables
x = coor(1);
y = coor(2);
z = coor(3);
L1 = link(1);
L2 = link(2);
L3 = link(3);
L4 = link(4); % is 0
L5 = link(5);

% Get Q1 rotation from xy position
Q1 = atan2(y,x);

% Get distance to effector in xy-plane
xy_Edist = sqrt(x^2 + y^2);

% Do Trigonometry to get Q2 and Q3
% Cosine Rule to find Q3
h = z - L1 + L5;
c = sqrt(xy_Edist^2 + h^2);
Q3 = acos((c^2 - L2^2 - L3^2)/(-2*L2*L3)) - pi;

% Q2 is sum of th1 and th2
th1 = atan2(h, xy_Edist);
th2 = asin(L3*sin(pi+Q3)/c); % Sine rule
Q2 = th1+th2;

% Get Q4 frorm Q2 and Q3
Q4 = -Q3-Q2;

% Output in degree
Q1 = rad2deg(Q1);
Q2 = rad2deg(Q2);
Q3 = rad2deg(Q3);
Q4 = rad2deg(Q4);
end

%% Cubic Polynomial
% Input: initial position and velocity, final position and velocity, t to complete task
% Output: cubic polynomial coefficients
function [a0 a1 a2 a3] = cubicpoly(x_i, xd_i, x_f, xd_f, t)
% Define cubic polynomial equation
syms a0 a1 a2 a3
eq1 = a0 == x_i;
eq2 = a1 == xd_i;
eq3 = a0 + a1*t + a2*t^2 + a3*t^3 == x_f;
eq4 = a1 + 2*a2*t + 3*a3*t^2 == xd_f;

% Solve for coefficients
[A,B] = equationsToMatrix([eq1, eq2, eq3, eq4], [a0, a1, a2, a3]);
coeff = linsolve(A,B);
a0 = coeff(1);
a1 = coeff(2);
a2 = coeff(3);
a3 = coeff(4);
end

%% Rotation Trajectory
% Input: reference orientation, effector orientation, both in frame 0 rotM
% Output: rotM to reference orientation in frame 0
function [R, theta_f] = orTrajectory(Rref, R_effector, w_i, w_f, t)
% Constant
density = 1; % How dense the interpolated theta should be
rotM = cell(1, density);

% Find relative rotation to Reffector from Rref
r = transpose(Rref) * R_effector;

% Find theta_f and k
theta_f = simplify(2 * acos(0.5 * sqrt(1 + r(1,1) + r(2,2) + r(3,3))));
k = [r(3,2)-r(2,3); r(1,3)-r(3,1); r(2,1)-r(1,2)]/(2*sin(theta_f));

% Interpolate theta
[a0 a1 a2 a3] = cubicpoly(0, w_i, theta_f, w_f, t);

index = 1;
theta = sym(zeros(1,density));
R = cell(1,density);
% Find rotM at each time instant
for i = [0:t/density:t] 
    theta(index) = a0 + a1*i + a2*i^2 + a3*i^3;
    e1 = k(1) * sin(theta(index)/2);
    e2 = k(2) * sin(theta(index)/2);
    e3 = k(3) * sin(theta(index)/2);
    e4 = cos(theta(index)/2);
    
    rotM{index} =   [1-2*e2^2-2*e3^2    2*(e1*e2-e3*e4) 2*(e1*e3+e2*e4);
                     2*(e1*e2+e3*e4)    1-2*e1^2-2*e3^2 2*(e2*e3-e1*e4);
                     2*(e1*e3-e2*e4)    2*(e2*e3+e1*e4) 1-2*e1^2-2*e2^2];
    
    R{index} = simplify(Rref*rotM{index});
    index = index + 1;
end
end

% Convert chess tile position to metres from robot
function [chess_x, chess_y] = chessPos2Metre(chess_pos)
% Constants
board_dimension = 0.30; % 30x30 cm^2
tile = board_dimension/8;
tile_center = tile/2; % Center of tile
offset_x = -0.1; % Robot Offset in x direction
offset_y = 4*tile; % Robot Offset in y direction

chess_x = -offset_x + (chess_pos(1)-1) * tile + tile_center; % Chess piece x position in metre
chess_y = -offset_y + (chess_pos(2)-1) * tile + tile_center; % Chess piece y position in metre
end

% Use 2 viapoints to move
function [v1, v2, vf, theta_v1, theta_v2, theta_f] = via2point(init_pos, final_pos, Rref, R_effector, t_tr, t_rot)
% Constant
height = 0.15; % 15 cm lift, so can go over King piece which is 6.3cm
z_i = 0.016; % Pawn piece is 3.1cm tall, go to center
zd_i = 0;
z_f = 0.016; % Pawn piece is 3.1cm tall, go to center
zd_f = 0;
w_i = 0;
w_f = 0;
syms t

% Positions
[x_i y_i] = chessPos2Metre(init_pos);
[x_f y_f] = chessPos2Metre(final_pos);

% Via point 1, directly above chess piece, only z will have movement
syms x_v1 xd_v1 y_v1 yd_v1 z_v1 zd_v1 w_v1
xd_i = 0;
x_v1 = x_i;
xd_v1 = 0; % Not moving in x
yd_i = 0;
y_v1 = y_i;
yd_v1 = 0; % Not moving in y
z_v1 = z_i + height;
zd_v1 = 0; % Stop at correct z position
w_v1 = 0;
[v1_x_a0 v1_x_a1 v1_x_a2 v1_x_a3] = cubicpoly(x_i, xd_i, x_v1, xd_v1, t_tr);
[v1_y_a0 v1_y_a1 v1_y_a2 v1_y_a3] = cubicpoly(y_i, yd_i, y_v1, yd_v1, t_tr);
[v1_z_a0 v1_z_a1 v1_z_a2 v1_z_a3] = cubicpoly(z_i, zd_i, z_v1, zd_v1, t_tr);
[R_v1, theta_v1] = orTrajectory(Rref, R_effector, w_i, w_v1, t_rot);
disp('Via Point 1, lifting up from initial position')
disp('z-axis path equation is,')
fprintf("%.3f t^3 + %.3f t^2 + %.3f t + %.3f\n", v1_z_a3, v1_z_a2, v1_z_a1, v1_z_a0)
disp('theta_f for Viapoint 1 is,')
disp(vpa(theta_v1,3))
v1_x(t) = v1_x_a3*t^3 + v1_x_a2*t^2 + v1_x_a1*t + v1_x_a0;
v1_y(t) = v1_y_a3*t^3 + v1_y_a2*t^2 + v1_y_a1*t + v1_y_a0;
v1_z(t) = v1_z_a3*t^3 + v1_z_a2*t^2 + v1_z_a1*t + v1_z_a0;

% Via point 2, directly above destination
syms x_v2 xd_v2 y_v2 yd_v2 z_v2 zd_v2 w_v2
x_v2 = x_f;
xd_v2 = 0; % Stop x movement when reach destination
y_v2 = y_f;
yd_v2 = 0; % Stop y movement when reach destination
z_v2 = z_v1;
zd_v2 = 0; % Not moving in z
w_v2 = 0;
[v2_x_a0 v2_x_a1 v2_x_a2 v2_x_a3] = cubicpoly(x_v1, xd_v1, x_v2, xd_v2, t_tr);
[v2_y_a0 v2_y_a1 v2_y_a2 v2_y_a3] = cubicpoly(y_v1, yd_v1, y_v2, yd_v2, t_tr);
[v2_z_a0 v2_z_a1 v2_z_a2 v2_z_a3] = cubicpoly(z_v1, zd_v1, z_v2, zd_v2, t_tr);
[R_v2, theta_v2] = orTrajectory(Rref, R_effector, w_v1, w_v2, t_rot);
disp('Via Point 2, moving to above destination,')
disp('x-axis path equation is')
fprintf("%.3f t^3 + %.3f t^2 + %.3f t + %.3f\n", v2_x_a3, v2_x_a2, v2_x_a1, v2_x_a0)
disp('y-axis path equation is')
fprintf("%.3f t^3 + %.3f t^2 + %.3f t + %.3f\n", v2_y_a3, v2_y_a2, v2_y_a1, v2_y_a0)
disp('theta_f for Viapoint 2 is,')
disp(vpa(theta_v2,3))
v2_x(t) = v2_x_a3*t^3 + v2_x_a2*t^2 + v2_x_a1*t + v2_x_a0;
v2_y(t) = v2_y_a3*t^3 + v2_y_a2*t^2 + v2_y_a1*t + v2_y_a0;
v2_z(t) = v2_z_a3*t^3 + v2_z_a2*t^2 + v2_z_a1*t + v2_z_a0;

% Final position, only move downwards
xd_f = 0;
yd_f = 0;
[f_x_a0 f_x_a1 f_x_a2 f_x_a3] = cubicpoly(x_v2, xd_v2, x_f, xd_f, t_tr);
[f_y_a0 f_y_a1 f_y_a2 f_y_a3] = cubicpoly(y_v2, yd_v2, y_f, yd_f, t_tr);
[f_z_a0 f_z_a1 f_z_a2 f_z_a3] = cubicpoly(z_v2, zd_v2, z_f, zd_f, t_tr);
[R_f, theta_f] = orTrajectory(Rref, R_effector, w_v2, w_f, t_rot);
disp('Final position, move down in z-axis to destination')
disp('z-axis path equation is')
fprintf("%.3f t^3 + %.3f t^2 + %.3f t + %.3f\n", f_z_a3, f_z_a2, f_z_a1, f_z_a0)
disp('theta_f for final position is,')
disp(vpa(theta_f,3))
vf_x(t) = f_x_a3*t^3 + f_x_a2*t^2 + f_x_a1*t + f_x_a0;
vf_y(t) = f_y_a3*t^3 + f_y_a2*t^2 + f_y_a1*t + f_y_a0;
vf_z(t) = f_z_a3*t^3 + f_z_a2*t^2 + f_z_a1*t + f_z_a0;

v1 = cell(1,3);
v1{1} = v1_x;
v1{2} = v1_y;
v1{3} = v1_z;
v2 = cell(1,3);
v2{1} = v2_x;
v2{2} = v2_y;
v2{3} = v2_z;
vf = cell(1,3);
vf{1} = vf_x;
vf{2} = vf_y;
vf{3} = vf_z;
end

%% Testing
% Q1 = 0; Q2 = 0; Q3 = 0;
% vpa(subs(R_direct{2},2)