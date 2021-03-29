%% Creating Rotations, Translations, Augmented Matrix
syms x y z angle
t = @(x,y,z) [x;y;z];
%t(1,2,3)                           % test

Rx =  @(angle)[1,0,0; 0, cos(angle),sin(angle); 0,-sin(angle),cos(angle)];
Rz = @(angle) [cos(angle),sin(angle),0; -sin(angle),cos(angle),0; 0,0,1];

% RzA = [Rx(angle), t(x,y,z); 0 0 0 1]           % test
RxA = @(angle,x,y,z) [Rx(angle), t(x,y,z); 0, 0, 0, 1];
RzA = @(angle,x,y,z) [Rz(angle), t(x,y,z); 0, 0, 0, 1];

%% Denavit Hartenberg
syms a alpha d theta

% T (i-1) --> i   =  Dx(a-1)*Rx(alpha-1)*Dz(d)*Rz(theta) 
transNext = @(a, alpha, d, theta) RxA(0,a,0,0)*RxA(alpha,0,0,0)*RzA(0,0,0,d)*RzA(theta,0,0,0);

%% DH Table to Transforms
% [a(i-1) alpha(i-1) d(i) theta(i)]

syms d1 th1 d2 th2 a2 d3 th3 a3 d4 th4 d5 th5 d6

DHtable =   [0 0 d1 th1; ...
            0 pi/2 d2 th2; ...
            a2 0 d3 th3; ...
            a3 0 d4 th4; ...
            0 -pi/2 -d5 th5;
            0 0 -d6 0];
final = eye(4);
for row = 1:length(DHtable)
    
    a = DHtable(row,1);
    alpha = DHtable(row,2);
    d = DHtable(row,3);
    theta = DHtable(row,4);
    
    fprintf("Matrix: %d to %d",row-1,row)
    transNext(a, alpha, d, theta)
    final = final*transNext(a, alpha, d, theta);
end
final


%% Inverse