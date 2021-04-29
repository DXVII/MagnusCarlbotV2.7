%% ---------------------------------------------------------------------------------------------------
%% -- Traj -------------------------------------------------------------------------------------------
%% ---------------------------------------------------------------------------------------------------

syms t xi xf
syms a0 a1 a2 a3 
syms b0 b1 b2 b3 
syms c0 c1 c2 c3

% t b/w 2-5 secs
t_sp = 5/3;

xi = [0; 0; 5];
xf = [5; 5; 5];
height = 10;
xHeight = [0; 0; height]
XB = xi + xHeight;
XC = xf + xHeight;

traj_AB = a0 + a1 .* t + a2 .* t.^2 + a3 .* t.^3;
traj_BC = b0 + b1 .* t + b2 .* t.^2 + b3 .* t.^3;
traj_CD = c0 + c1 .* t + c2 .* t.^2 + c3 .* t.^3;

vel_AB = diff(traj_AB,t);
vel_BC = diff(traj_BC,t);
vel_CD = diff(traj_CD,t);

syms XA XB XC XD VA VB VC VD

t0 = 0; 
t1 = t_sp;
t2 = 2*t_sp; 
t3 = 3*t_sp;

XA = xi;
XD = xf;

% position
f1 = XA == subs(traj_AB, t, t0)      % begin xi
 
f2 = XB == subs(traj_AB, t, t1)
f3 = XB == subs(traj_BC, t, t1) 

f4 = XC == subs(traj_BC, t, t2)
f5 = XC == subs(traj_CD, t, t2)

f6 = XD == subs(traj_CD, t, t3)      % end xf





% velocity
% f7 = VA == subs(vel_AB, t, t0)      % begin vi = 0
f7 = 0 == subs(vel_AB, t, t0)         


f8 = VB == subs(vel_AB, t, t1)
f9 = VB == subs(vel_BC, t, t1) 

f10 = VC == subs(vel_BC, t, t2)
f11 = VC == subs(vel_CD, t, t2)

f12 = 0 == subs(vel_CD, t, t3)
% f12 = VD == subs(vel_CD, t, t3)      % end vf = 0

VA = 0;
VD = 0;


%{
    
    f01 =   XA == a0
    f02 =   XB == a0 + 1.6667 * a1 + 2.7778 * a2 + 4.6296 * a3
    f03 =   XB == b0 + 1.6667 * b1 + 2.7778 * b2 + 4.6296 * b3
    f04 =   XC == b0 + 3.3333 * b1 + 11.1111 * b2 + 37.0370 * b3
    f05 =   XC == c0 + 3.3333 * c1 + 11.1111 * c2 + 37.0370 * c3
    f06 =   XD == c0 + 5 * c1 + 25 * c2 + 125 * c3

    f07 =   VA == a1
    f08 =   VB == a1 + 3.3333 * a2 + 8.3333 * a3
    f09 =   VB == b1 + 3.3333 * b2 + 8.3333 * b3
    f10 =   VC == b1 + 6.6667 * b2 + 33.3333 * b3
    f11 =   VC == c1 + 6.6667 * c2 + 33.3333 * c3
    f12 =   VD == c1 + 10 * c2 + 75 * c3
   
%}

eqns = [f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12];
coefvars = [a0, a1, a2, a3, b0, b1, b2, b3, c0, c1, c2, c3];
[A, b] = equationsToMatrix(eqns, coefvars)
sols = A*b

a0 = sols(1) 
a1 = sols(2) 
a2 = sols(3) 
a3 = sols(4) 

b0 = sols(5) 
b1 = sols(6) 
b2 = sols(7) 
b3 = sols(8)
 
c0 = sols(9) 
c1 = sols(10) 
c2 = sols(11) 
c3 = sols(12) 

%{
    
a0 = XA
a1 = XA + 4.4444 * XB + 4.6296 * XC
a2 = 2.7778 * VA + 4.6296 * VB + XC + 1.6667 * XD
a3 = 11.1111 * VA + 37.0370 * VB + XC + 3.3333 * XD
b0 = VB + 14.4444 * VC + 37.0370 * VD
b1 = VB + 30 * VC + 125 * VD
b2 = XB
b3 = 4.3333 * XB + 8.3333 * XC
c0 = 3.3333 * VA + 8.3333 * VB + XD
c1 = 6.6667 * VA + 33.3333 * VB + XD
c2 = 7.6667 * VC + 33.3333 * VD
c3 = 11 * VC + 75 * VD


TODO:
define XB and XC as a vert height about XA (xi) and XD (xf) by height

%}

%% ---------------------------------------------------------------------------------------------------
%% -- Pose -------------------------------------------------------------------------------------------
%% ---------------------------------------------------------------------------------------------------

%% Creating Rotations, Translations, Augmented Matrix
syms x y z angle

% Translation
t = @(x, y, z) [x; y; z];

% Rotation
Rx = @(angle)[1, 0, 0; 0, cos(angle), sin(angle); 0, -sin(angle), cos(angle)];
Rz = @(angle) [cos(angle), sin(angle), 0; -sin(angle), cos(angle), 0; 0, 0, 1];

% Augment
RxA = @(angle, x, y, z) [Rx(angle), t(x, y, z); 0, 0, 0, 1];
RzA = @(angle, x, y, z) [Rz(angle), t(x, y, z); 0, 0, 0, 1];

%% Part 1: Denavit Hartenberg Method
syms a alpha d theta

% Transform Chain function:  T (i-1) --> i   =  Dx(a-1)*Rx(alpha-1)*Dz(d)*Rz(theta)
transNext = @(a, alpha, d, theta) RxA(0, a, 0, 0) * RxA(alpha, 0, 0, 0) * RzA(0, 0, 0, d) * RzA(theta, 0, 0, 0);

% DH Table to Transforms
% [a(i-1) alpha(i-1) d(i) theta(i)]
syms d1 th1 d2 th2 a2 d3 th3 a3 d4 th4 d5 th5 d6
DHtable = [0 0 d1 th1; ... % frame 0-1: XY shoulder swivel
            0 pi / 2 d2 th2; ... % frame 1-2: XZ bicep elevation
            a2 0 d3 th3; ... % frame 2-3: XZ elbow descent
            a3 0 d4 th4; ... % frame 3-4: XZ vertical wrist (constraint: point to ground)
            0 pi / 2 d5 th5; ... % frame 4-5: XY wrist torsional twist (constraint: point foward)
            0 0 d6 0]; % frame 5-6: Wrist to Gripper distance

%% Generate Poses
final = eye(4); % being DH chain

f_final = length(DHtable); % code tings: simplify end of array

% frame 1 to 4 chosen due to relational constrains of frames 4 and onwards (discussed in report)
frame_start = 1; % code tings: start of transform chain
frame_stop = 6; % code tings: end of transform chain

translations = sym([0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0]);

for row = frame_start:1:frame_stop
    a = DHtable(row, 1);
    alpha = DHtable(row, 2);
    d = DHtable(row, 3);
    theta = DHtable(row, 4);

    % fprintf("Matrix: %d to %d", row - 1, row);
    tmpMat(1:4, 1:4) = transNext(a, alpha, d, theta);
    positions(:, row) = tmpMat(1:3, 4);
    % summation of positions from "shoulder joint"
    rotStart = (row * 4 - 3);
    rotEnd = (row * 4 - 1);
    rotations(rotStart:rotEnd, 1:3) = tmpMat(1:3, 1:3);
    % invrotations(rotStart:rotEnd, 1:3) = transpose(tmpMat(1:3, 1:3));

end

%% Rotations
% rotations     --> R01  --> getRotMat(1, rotations)      --> R0'1'
% invrotations  --> R_10 --> getRotMat(1, invrotations)   --> R_'1'0
R01 = getRotMat(1, rotations);
R12 = getRotMat(2, rotations);
R23 = getRotMat(3, rotations);
R34 = getRotMat(4, rotations);
R45 = getRotMat(5, rotations);
R56 = getRotMat(6, rotations);

R02 = R01 * R12;
R03 = R01 * R12 * R23;
R04 = R01 * R12 * R23 * R34;
R05 = R01 * R12 * R23 * R34 * R45;
R06 = R05;


%% Stuff
% only need R45

R0F = R45;
theta_f = 2*acosd(0.5*sqrt(1 + R0F(1,1) + R0F(2,2) + R0F(3,3)))

k_hat = (1 / (2 * sind(theta_f))) * [(R0F(3,2)-R0F(2,3)); (R0F(1,3)-R0F(3,1)); (R0F(2,1)-R0F(1,2))]
kx = k_hat(1);
ky = k_hat(2);
kz = k_hat(3);




syms p0 p1 p2 p3 t
f_theta = p0 + p1.*t + p2.*t.^2 + p3.*t.^3  % begins at 0  =>  theta_f at the end of time given
e1 = kx * sind(theta_f/2);
e2 = ky * sind(theta_f/2);
e3 = kz * sind(theta_f/2);
e4 = cosd(theta_f/2);



Rit = [ (1-2*e2^2-2*e3^2)   ,   2*(e1*e2-e3*e4)     ,   2*(e1*e3+e2*e4) ;...
        2*(e1*e2-e3*e4)     ,   (1-2*e1^2-2*e3^2)   ,   2*(e2*e3-e1*e4) ;...
        2*(e1*e3+e2*e4)     ,   2*(e2*e3-e1*e4)     ,   (1-2*e1^2-2*e2^2)
      ];

R0t = R04 * Rit;


%{
    a0 = XA
    a1 = XA + 4.4444 * XB + 4.6296 * XC
    a2 = 4.6296 * VB + XC + 1.6667 * XD
    a3 = 37.0370 * VB + XC + 3.3333 * XD
    b0 = VB + 14.4444 * VC
    b1 = VB + 30 * VC
    b2 = XB
    b3 = 4.3333 * XB + 8.3333 * XC
    c0 = 8.3333 * VB + XD
    c1 = 33.3333 * VB + XD
    c2 = 7.6667 * VC
    c3 = 11 * VC



    theta_f = 114.5916 * acos(0.5000 * (cos(th5) + 1)^(1/2))
    th5 = th1


    k_hat =
    k_x = -(0.5000 * (cos(th5) + 1)) / sin(2 * acos(0.5000 * (cos(th5) + 1)^(1/2)))
    k_y = -(0.5000 * sin(th5)) / sin(2 * acos(0.5000 * (cos(th5) + 1)^(1/2)))
    k_z = -(0.5000 * sin(th5)) / sin(2 * acos(0.5000 * (cos(th5) + 1)^(1/2)))

%}


function RotMat = getRotMat(num, rotations)
    rotStart = (num * 4 - 3);
    rotEnd = (num * 4 - 1);
    RotMat = rotations(rotStart:rotEnd, 1:3);
end
