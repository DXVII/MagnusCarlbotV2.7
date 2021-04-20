%% Creating Rotations, Translations, Augmented Matrix
syms x y z angle

% Translation
t = @(x,y,z) [x;y;z];                       

% Rotation
Rx =  @(angle)[1,0,0; 0, cos(angle),sin(angle); 0,-sin(angle),cos(angle)];
Rz = @(angle) [cos(angle),sin(angle),0; -sin(angle),cos(angle),0; 0,0,1];

% Augment
RxA = @(angle,x,y,z) [Rx(angle), t(x,y,z); 0, 0, 0, 1];
RzA = @(angle,x,y,z) [Rz(angle), t(x,y,z); 0, 0, 0, 1];

%% Part 1: Denavit Hartenberg Method
syms a alpha d theta

% Transform Chain function:  T (i-1) --> i   =  Dx(a-1)*Rx(alpha-1)*Dz(d)*Rz(theta) 
transNext = @(a, alpha, d, theta) RxA(0,a,0,0)*RxA(alpha,0,0,0)*RzA(0,0,0,d)*RzA(theta,0,0,0);


% DH Table to Transforms
% [a(i-1) alpha(i-1) d(i) theta(i)]
syms d1 th1 d2 th2 a2 d3 th3 a3 d4 th4 d5 th5 d6
DHtable =   [0 0 d1 th1; ...            % frame 0-1: XY shoulder swivel
            0 pi/2 d2 th2; ...          % frame 1-2: XZ bicep elevation
            a2 0 d3 th3; ...            % frame 2-3: XZ elbow descent
            a3 0 d4 th4; ...            % frame 3-4: XZ vertical wrist (constraint: point to ground)
            0 pi/2 d5 th5; ...          % frame 4-5: XY wrist torsional twist (constraint: point foward)
            0 0 d6 0];                  % frame 5-6: Wrist to Gripper distance 


%% Generate Poses
final = eye(4);                 % being DH chain

f_final = length(DHtable);      % code tings: simplify end of array

% frame 1 to 4 chosen due to relational constrains of frames 4 and onwards (discussed in report)
frame_start = 1; % code tings: start of transform chain
frame_stop = 6; % code tings: end of transform chain

translations = sym([0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0]);

for row = frame_start:1:frame_stop
    a = DHtable(row, 1);
    alpha = DHtable(row, 2);
    d = DHtable(row, 3);
    theta = DHtable(row, 4);

    fprintf("Matrix: %d to %d", row - 1, row)
    tmpMat(1:4, 1:4) = transNext(a, alpha, d, theta)
    translations(row, :) = transpose(tmpMat(1:3, 4));
    
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

R_20 = inv(R02);
R_30 = inv(R03);
R_40 = inv(R04);
R_50 = inv(R05);
R_60 = R_50;

%% Joint Torques - Big = 16.5 kg*cm 
% Weight bearing motors at "shoulder", "elbow" and "wrist" (maybe).
% wrist may have "no torque" as it is always pointing down, does not lift
% anything

% Begin with finding tau_3
% me = end effector mass

syms Me Mm3 Mm2 Mm1 Mc1 Mc2 Mc3 
syms Mmb
% gravity
g = 9.81;
gzh = [0;0;g];
MH = Me + Mm3 +Mc3 + Mm2

FG_H = MH*gzh;      % hand
FG_c2 = Mc2*gzh    % forearm
FG_mb = Mmb*gzh;    % elbow
FG_c1 = Mc1*gzh;    % bicep

% translation - lengths
% l1        full bicep 
% l2        full forearm 
% l2/2      COM bicep
% l2/2      COM forearm

l1 = a2;
l2 = a3;

% dim lengths
rAC1_f1 = [l1/2; 0; 0];
rAB_f1 = [l1; 0; 0];
rBC2_f2 = [l2/2; 0; 0];
rBH_f2 = [l2; 0; 0];

%% Geometric r cross F method (frame 0)

% tau 2
rBH_f0 = R_20 * rBH_f2;
rBC2_f0 = R_20 * rBC2_f2

tau_2 = cross(rBC2_f0,FG_c2) + cross(rBH_f0,FG_H)

% tau 1
rAC1_f0 = R_10 * rAC1_f1;
rAB_f0 = R_10 * rAB_f1;
rAC2_f0 = rAB_f0 + R_20 * rBC2_f2; 
rAH_f0 = rAB_f0 + R_20 * rBH_f2;

tau_1 = cross(rAC1_f0,FG_c1) + cross(rAB_f0,FG_mb) + cross(rAC2_f0,FG_c2) + cross(rAH_f0,FG_H)

% insert masses and insert lengths

function RotMat = getRotMat(num, rotations)
    rotStart = (num * 4-3);
    rotEnd = (num * 4-1);

    RotMat = rotations(rotStart:rotEnd, 1:3);
end