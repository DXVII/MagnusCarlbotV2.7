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
a2 = 0.6;       % bicep length
a3 = 0.5;       % forearm length

% joints
d6 = 0.025;     % gripper contact length
d5 = 0.10;      % wrist hand length
d4 = 0;
d3 = 0;
d2 = 0;

% Appendix 1)

% iteratively plotting task space points with all possible angles (joint space possibilities)
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
plot(x,z,'o')
hold on;
plot(sideView_x,sideView_z,'-r','LineWidth',2)

title('X vs Z (measuring reachable height)')
xlabel("X distance (m)");
ylabel("Z distance (m)");
legend('Reachable space','','Chessboard')


% Robot Arm Top View 
figure
plot(x,y,'o')
hold on;
plot(topView_x,topView_y,'-r','LineWidth',2)

title('X vs Y(measuring top down 2D reachable area)')
xlabel("X distance (m)");
ylabel("Y distance (m)");
legend('Reachable space','','Chessboard')


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