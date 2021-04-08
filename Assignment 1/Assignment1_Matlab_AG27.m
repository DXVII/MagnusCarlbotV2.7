%% Robotic Systems Project 1 - Code:
% Group 27: David Pham, Pamela Kong, Thomas Pimenta


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
frame_stop = 4;             % code tings: end of transform chain


for row = frame_start:1:frame_stop    
    a = DHtable(row,1);
    alpha = DHtable(row,2);
    d = DHtable(row,3);
    theta = DHtable(row,4);
    
    fprintf("Matrix: %d to %d",row-1,row)
    tmpMat(1:4,1:4) = transNext(a, alpha, d, theta)
    final = final*transNext(a, alpha, d, theta);
end
final                                                   % final Augmented Matrix
finalTrans = simplify(final((1:3),4))                   % x,y,z postion equations 
%{
    Results of DH Method (final_Trans)
    d2*sin(th1) - a3*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + d3*sin(th1) + d4*sin(th1) + a2*cos(th1)*cos(th2)
    a3*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) + d2*cos(th1) + d3*cos(th1) + d4*cos(th1) - a2*cos(th2)*sin(th1)
                                                                                             d1 + a3*sin(th2 + th3) + a2*sin(th2)-(d5+d6)
    % chucked -(d5+d6) offset into calculated z equation
%}

%% Plot Reachable
syms d1 th1 d2 th2 a2 d3 th3 a3 d4 th4 d5 th5 d6
theta1 = 0;
theta2 = 0;
theta3 = 0;

% Motor Angle Limits
minth1 = 0;
maxth1 = 90;

minth2 = 10;
maxth2 = 90;

minth3 = 0;
maxth3 = 150;

% iteration stepsize
step = 10;
val1 = (maxth1-minth1)/step;
val2 = (maxth2-minth2)/step;
val3 = (maxth3-minth3)/step;

% code tings: intialisation
x = zeros(val1*val2*val3);
y = zeros(val1*val2*val3);
z = zeros(val1*val2*val3);
index = 1;

% measurements in metres
% arm lengths 
d1 = 0.05;      % shoulder length
a2 = 0.5;       % bicep length
a3 = 0.4;       % forearm length

% joints
d6 = 0.025;     % gripper contact length
d5 = 0.10;      % wrist hand length
d4 = 0;
d3 = 0;
d2 = 0;

% Appendix 1)

% Iteratively plotting task space points with all possible angles (joint space possibilities)
for i = minth1:step:maxth1
    for j = minth2:step:maxth2
        for k = minth3:step:maxth3
            ri = deg2rad(i);
            rj = deg2rad(j);
            rk = -deg2rad(k);
            
            x(index) = d2 * sin(ri) - a3 * (cos(ri) * sin(rj) * sin(rk) - cos(ri) * cos(rj) * cos(rk)) + d3 * sin(ri) + d4 * sin(ri) + a2 * cos(ri) * cos(rj);
            y(index) = a3 * (sin(ri) * sin(rj) * sin(rk) - cos(rj) * cos(rk) * sin(ri)) + d2 * cos(ri) + d3 * cos(ri) + d4 * cos(ri) - a2 * cos(rj) * sin(ri);
            z(index) = d1 + a3 * sin(rj + rk) + a2 * sin(rj) -(d5 + d6);
            index = index + 1;
        end
    end
end

topView_x = [0.2 0.2 0.6 0.6 0.2];
topView_y = [-0.2 -0.6 -0.6 -0.2 -0.2];

sideView_x = [0.2 0.6];
sideView_z = [0 0];


% Robot Arm Side View
figure
plot(sideView_x,sideView_z,'-','Color','#F59D49','LineWidth',2)
hold on;
plot(x,z,'x','Color','#4485F5','MarkerSize',8)

title('X vs Z (measuring reachable height)')
xlabel("X distance (m)");
ylabel("Z distance (m)");
legend('Chessboard','Reachable space')


% Robot Arm Top View 
figure
plot(topView_x,topView_y,'-','Color','#F59D49','LineWidth',2)
hold on;
plot(x,y,'x','Color','#4485F5','MarkerSize',8)

title('X vs Y (measuring top down 2D reachable area)')
xlabel("X distance (m)");
ylabel("Y distance (m)");
legend('Chessboard','Reachable space')

%% Inverse Kinematics - Chain Inverse Kinematics --> solve q1 as tanInv(y/x) --> substitute q1 into 
syms d1 th1 d2 th2 a2 d3 th3 a3 d4 th4 d5 th5 d6
syms x y z
f1 = d2*sin(th1) - a3*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + d3*sin(th1) + d4*sin(th1) + a2*cos(th1)*cos(th2);
f2 = a3*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) + d2*cos(th1) + d3*cos(th1) + d4*cos(th1) - a2*cos(th2)*sin(th1);
f3 = d1 + a3*sin(th2 + th3) + a2*sin(th2)-(d5+d6);

% Simplifying negligible lengths --> reduce plot lag
fx = subs(f1,{d2,d3,d4},{0,0,0});
fy = subs(f2,{d2,d3,d4},{0,0,0});
fz = subs(f3,{d2,d3,d4},{0,0,0});

%{
fx = a2*cos(th1)*cos(th2) - a3*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))
fy = a3*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) - a2*cos(th2)*sin(th1)
fz = d1 - d5 - d6 + a3*sin(th2 + th3) + a2*sin(th2)
%}


% Linearise
% To find valid equilibria

% equilibrium points, (to be found)
eq_th1 = pi/4;
eq_th2 = pi/3;
eq_th3 = 4*pi/3;
equib = {eq_th1, eq_th2, eq_th3};

thetas = {th1, th2, th3};

% taylor poly tings
dfxdth1 = diff(fx, th1);
dfxdth2 = diff(fx, th2);
dfxdth3 = diff(fx, th3);

dfydth1 = diff(fy, th1);
dfydth2 = diff(fy, th2);
dfydth3 = diff(fy, th3);

dfzdth1 = diff(fz, th1);
dfzdth2 = diff(fz, th2);
dfzdth3 = diff(fz, th3);


% taylor polynomial --> linear
lin_x = x == subs(dfxdth1, thetas, equib) * th1 + subs(dfxdth2, thetas, equib) * th2 + subs(dfxdth3, thetas, equib) * th3;
lin_y = y == subs(dfydth1, thetas, equib) * th1 + subs(dfydth2, thetas, equib) * th2 + subs(dfydth3, thetas, equib) * th3;
lin_z = z == subs(dfzdth1, thetas, equib) * th1 + subs(dfzdth2, thetas, equib) * th2 + subs(dfzdth3, thetas, equib) * th3;

% Idea: linearise to convert system of equations into matrix 
% [a1 a2 a3]   [th1]   [x]
% [b1 b2 b3] * [th2] = [y]
% [c1 c2 c3]   [th3]   [z]
% 
% coeffs*Theta = Coords
% Thetas = Inv(coeffs)*Coords  % bam inv kinematics

[cs, Coords] = equationsToMatrix([lin_x, lin_y, lin_z], [th1, th2, th3]);

Thetas = inv(cs)*Coords;
th1 = Thetas(1)
th2 = Thetas(2)
th3 = Thetas(3)


%% Appendix Plot Equations

% appendix 1) Test Equations for plotting

% Original
% fx = d2 * sin(th1) - a3 * (cos(th1) * sin(th2) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + d3 * sin(th1) + d4 * sin(th1) + a2 * cos(th1) * cos(th2);
% fy = a3 * (sin(th1) * sin(th2) * sin(th3) - cos(th2) * cos(th3) * sin(th1)) + d2 * cos(th1) + d3 * cos(th1) + d4 * cos(th1) - a2 * cos(th2) * sin(th1);
% fz = d1 + a3 * sin(th2 + th3) + a2 * sin(th2) - d5;

% x(index) = d2 * sin(ri) - a3 * (cos(ri) * sin(rj) * sin(rk) - cos(ri) * cos(rj) * cos(rk)) + d3 * sin(ri) + d4 * sin(ri) + a2 * cos(ri) * cos(rj);
% y(index) = a3 * (sin(ri) * sin(rj) * sin(rk) - cos(rj) * cos(rk) * sin(ri)) + d2 * cos(ri) + d3 * cos(ri) + d4 * cos(ri) - a2 * cos(rj) * sin(ri);
% z(index) = d1 + a3 * sin(rj + rk) + a2 * sin(rj) - d5;



% Simplifying negligible lengths --> reduce plot lag
% f1 = subs(fx,{d2,d3,d4},{0,0,0});
% f2 = subs(fy,{d2,d3,d4},{0,0,0});
% f3 = subs(fz,{d2,d3,d4},{0,0,0});

% Simplified
% f1 = a2 * cos(th1) * cos(th2) - a3 * (cos(th1) * sin(th2) * sin(th3) - cos(th1) * cos(th2) * cos(th3))
% f2 = a3 * (sin(th1) * sin(th2) * sin(th3) - cos(th2) * cos(th3) * sin(th1)) - a2 * cos(th2) * sin(th1)
% f3 = d1 - d5 + a3 * sin(th2 + th3) + a2 * sin(th2)

% x(index) = a2 * cos(ri) * cos(rj) - a3 * (cos(ri) * sin(rj) * sin(rk) - cos(ri) * cos(rj) * cos(rk));
% y(index) = a3 * (sin(ri) * sin(rj) * sin(rk) - cos(rj) * cos(rk) * sin(ri)) - a2 * cos(rj) * sin(ri);
% z(index) = d1 - d5 + a3 * sin(rj + rk) + a2 * sin(rj);
