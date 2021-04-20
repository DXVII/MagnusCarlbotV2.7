%%Variables
syms th1 th2 th3 th4 th5
%Angle Formulae
Rx =  @(angle)[1,0,0; 0, cos(angle),sin(angle); 0,-sin(angle),cos(angle)];
Rz = @(angle) [cos(angle),sin(angle),0; -sin(angle),cos(angle),0; 0,0,1];

%Rotations
Rx90 = round(Rx(pi/2),0);
R01 = Rx(0)*Rz(th1);
% R12 = Rx(pi/2)*Rz(th2);
R12 = Rx90*Rz(th2);
R23 = Rx(0)*Rz(th3);
R34 = Rx(0)*Rz(th4);
% R45 = Rx(pi/2)*Rz(th5);
R45 = Rx90*Rz(th5);
R56 = eye(3);

R02 = R01*R12;
R03 = R01*R12*R23;
R04 = R01*R12*R23*R34;
R05 = R01*R12*R23*R34*R45;
R06 = R05;

%%Calculations
z = [0;0;1];
z1 = R01*z;
z2 = R02*z;
z3 = R03*z;
z4 = R04*z;
z5 = R05*z;
z6 = R06*z;

Z_mat = [z1 z2 z3 z4 z5 z6];
% Z_mat_fin = vpa(Z_mat,3)
%% Linear
t = @(x,y,z) [x;y;z];
RxA = @(angle,x,y,z) [Rx(angle), t(x,y,z); 0, 0, 0, 1];
RzA = @(angle,x,y,z) [Rz(angle), t(x,y,z); 0, 0, 0, 1];

syms a alpha d theta
transNext = @(a, alpha, d, theta) RxA(0,a,0,0)*RxA(alpha,0,0,0)*RzA(0,0,0,d)*RzA(theta,0,0,0);

syms d1 th1 d2 th2 a2 d3 th3 a3 d4 th4 d5 th5 d6
DHtable =   [0 0 d1 th1; ...            % frame 0-1: XY swivel
            0 pi/2 d2 th2; ...          % frame 1-2: XZ bicep elevation
            a2 0 d3 th3; ...            % frame 2-3: XZ elbow descent
            a3 0 d4 th4; ...            % frame 3-4: XZ vertical wrist (constraint: point to ground)
            0 pi/2 d5 th5; ...          % frame 4-5: XY wrist torsional twist (constraint: point foward)
            0 0 d6 0];                  % frame 5-6: Wrist Gripper to distance 

final = eye(4);             % being DH chain

f_final = length(DHtable);  % code tings: simplify end of array 

% frame 1 to 4 chosen due to relational constrains of frames 4 and onwards (discussed in report)
frame_start = 1;            % code tings: start of transform chain
frame_stop = 6;             % code tings: end of transform chain

translations = sym([0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0]);

for row = frame_start:1:frame_stop    
    a = DHtable(row,1);
    alpha = DHtable(row,2);
    d = DHtable(row,3);
    theta = DHtable(row,4);
    
%     fprintf("Matrix: %d to %d",row-1,row)
    tmpMat(1:4,1:4) = transNext(a, alpha, d, theta);
    translations(row,:) = transpose(tmpMat(1:3,4));
    final = final*transNext(a, alpha, d, theta);
end
%Creating vectors between frames
r01_0 = translations(1,:);
r12_1 = translations(2,:);
r23_2 = translations(3,:);
r34_3 = translations(4,:);
r45_4 = translations(5,:);
r56_5 = translations(6,:);

%Creating vectors from frame to end effector in 0 frame orientation

r56_0 = R05*r56_5';
r46_0 = R04*r45_4' + r56_0;
r36_0 = R03*r34_3' + r46_0;
r26_0 = R02*r23_2' + r36_0;
r16_0 = R01*r12_1' + r26_0;
% r06_0 = r01_0' + r26_0; %unnecessary

Jv = [cross(z1,r16_0) cross(z2,r26_0) cross(z3,r36_0) cross(z4,r46_0) cross(z5,r56_0)];

%% Joint Wrench calcs

r2c2_2 = r23_2/2;
r3c3_3 = r34_3/2;
r2c2_0 = R02*r2c2_2';
r3c3_0 = R03*r3c3_3';

zero = [0;0;0];
syms Mm1 Mm2 Mm3 Mm4 Mm5 Mm6 Ma2 Ma3 ME Mp

 Jvc1 = [cross(z1,R01*r12_1'+r2c2_0) cross(z2,r2c2_0) zero zero zero]';
 Jvc2 = [cross(z1,R01*r12_1' + R02*r23_2' + r3c3_0) cross(z2,R02*r23_2' + r3c3_0) cross(z3,r3c3_0) zero zero]';

%  Jvm1 = 
Jvm2 = [cross(z1,R01*r12_1') zero zero zero zero]';
Jvm3 = [cross(z1,R01*r12_1'+R02*r23_2') cross(z2,R02*r23_2') zero zero zero]';
Jvm4 = [cross(z1,R01*r12_1'+R02*r23_2' + R03*r34_3') cross(z2,R02*r23_2'+R03*r34_3') cross(z3,R03*r34_3') zero zero]';
Jvm5 = [cross(z1,R01*r12_1'+R02*r23_2' + R03*r34_3' + R04*r45_4') cross(z2,R02*r23_2'+R03*r34_3' + R04*r45_4') cross(z3,R03*r34_3'+R04*r45_4') cross(z4,R04*r45_4') zero]';
Jvm6 = [cross(z1,R01*r12_1'+R02*r23_2' + R03*r34_3' + R04*r45_4' + R05*r56_5') cross(z2,R02*r23_2'+R03*r34_3' + R04*r45_4' + R05*r56_5') cross(z3,R03*r34_3'+R04*r45_4'+R05*r56_5') cross(z4,R04*r45_4'+R05*r56_5') cross(z5,R05*r56_5')]';

JvE = transpose(Jv);

FE = [0;0;-Mp];
Fc1 = [0;0;-Ma2];
Fc2 = [0;0;-Ma3];
Fm1 = [0;0;-Mm1];
Fm2 = [0;0;-Mm2];
Fm3 = [0;0;-Mm3];
Fm4 = [0;0;-Mm4];
Fm5 = [0;0;-Mm5];
Fm6 = [0;0;-Mm6];

T = JvE*FE + Jvc1*Fc1 + Jvc2*Fc2 + Jvm2*Fm2 + Jvm3*Fm3 + Jvm4*Fm4 + Jvm5*Fm5 + Jvm6*Fm6
