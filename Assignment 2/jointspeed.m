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
