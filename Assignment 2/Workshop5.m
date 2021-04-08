%% Workshop 5
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


final = eye(4);             % being DH chain

f_final = length(DHtable);  % code tings: simplify end of array 

% frame 1 to 4 chosen due to relational constrains of frames 4 and onwards (discussed in report)
frame_start = 1;            % code tings: start of transform chain
frame_stop = 5;             % code tings: end of transform chain


for row = frame_start:1:f_final    
    a = DHtable(row,1);
    alpha = DHtable(row,2);
    d = DHtable(row,3);
    theta = DHtable(row,4);
    
    fprintf("Matrix: %d to %d",row-1,row)
    tmpMat(1:4,1:4) = transNext(a, alpha, d, theta)
    obj{row} = tmpMat;
    final = final*transNext(a, alpha, d, theta);
end
final;                                                      % final Augmented Matrix
finalTrans = simplify(final((1:3),4));                      % x,y,z postion equations 
%{
Results of DH Method (final_Trans)
    f1 = 0.5000 * a3 * cos(th2 - th1 + th3) + 0.5000 * d5 * sin(th1 + th2 + th3 + th4) + 0.5000 * d6 * sin(th1 + th2 + th3 + th4) + 0.5000 * a2 * cos(th1 + th2) + d2 * sin(th1) + d3 * sin(th1) + d4 * sin(th1) + 0.5000 * d5 * sin(th2 - th1 + th3 + th4) + 0.5000 * d6 * sin(th2 - th1 + th3 + th4) + 0.5000 * a2 * cos(th1 - th2) + 0.5000 * a3 * cos(th1 + th2 + th3)
    f2 = d2 * cos(th1) + d3 * cos(th1) + d4 * cos(th1) - a3 * cos(th2 + th3) * sin(th1) - a2 * cos(th2) * sin(th1) - d5 * sin(th2 + th3 + th4) * sin(th1) - d6 * sin(th2 + th3 + th4) * sin(th1)
    f3 = d1 + a3 * sin(th2 + th3) + a2 * sin(th2) - d5 * cos(th2 + th3 + th4) - d6 * cos(th2 + th3 + th4)
%}

%% Jacobian
syms d1 th1 d2 th2 a2 d3 th3 a3 d4 th4 d5 th5 d6
syms x y z

f1 = finalTrans(1);
f2 = finalTrans(2);
f3 = finalTrans(3);

% Simplifying negligible lengths --> reduce plot lag
fx = subs(f1,{d2,d3,d4},{0,0,0});
fy = subs(f2,{d2,d3,d4},{0,0,0});
fz = subs(f3,{d2,d3,d4},{0,0,0});

%{
    fx = 0.5000 * a3 * cos(th2 - th1 + th3) + 0.5000 * d5 * sin(th1 + th2 + th3 + th4) + 0.5000 * d6 * sin(th1 + th2 + th3 + th4) + 0.5000 * a2 * cos(th1 + th2) + 0.5000 * d5 * sin(th2 - th1 + th3 + th4) + 0.5000 * d6 * sin(th2 - th1 + th3 + th4) + 0.5000 * a2 * cos(th1 - th2) + 0.5000 * a3 * cos(th1 + th2 + th3)
    fy = - a3 * cos(th2 + th3) * sin(th1) - a2 * cos(th2) * sin(th1) - d5 * sin(th2 + th3 + th4) * sin(th1) - d6 * sin(th2 + th3 + th4) * sin(th1)
    fz = d1 + a3 * sin(th2 + th3) + a2 * sin(th2) - d5 * cos(th2 + th3 + th4) - d6 * cos(th2 + th3 + th4)
%}
thetas = {th1, th2, th3, th4, th5};

% Manual Jacobian tings
dfxdth1 = diff(fx, th1);
dfxdth2 = diff(fx, th2);
dfxdth3 = diff(fx, th3);
dfxdth4 = diff(fx, th4);
dfxdth5 = diff(fx, th5);

dfydth1 = diff(fy, th1);
dfydth2 = diff(fy, th2);
dfydth3 = diff(fy, th3);
dfydth4 = diff(fy, th4);
dfydth5 = diff(fy, th5);

% Only care about this for angle rotations
dfzdth1 = diff(fz, th1);
dfzdth2 = diff(fz, th2);
dfzdth3 = diff(fz, th3);
dfzdth4 = diff(fz, th4);
dfzdth5 = diff(fz, th5);


