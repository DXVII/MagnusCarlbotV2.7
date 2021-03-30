%% Creating Rotations, Translations, Augmented Matrix
syms x y z angle

t = @(x,y,z) [x;y;z];
%t(1,2,3)                           % test

Rx =  @(angle)[1,0,0; 0, cos(angle),sin(angle); 0,-sin(angle),cos(angle)];
Rz = @(angle) [cos(angle),sin(angle),0; -sin(angle),cos(angle),0; 0,0,1];
% invRx =  @(angle)[1,0,0; 0, cos(angle),sin(angle); 0,-sin(angle),cos(angle)]';
% invRz = @(angle) [cos(angle),sin(angle),0; -sin(angle),cos(angle),0; 0,0,1]';


RxA = @(angle,x,y,z) [Rx(angle), t(x,y,z); 0, 0, 0, 1];
RzA = @(angle,x,y,z) [Rz(angle), t(x,y,z); 0, 0, 0, 1];
% RzA = [Rx(angle), t(x,y,z); 0 0 0 1]           % test

% invRxA = @(angle,x,y,z) [invRx(angle), t(x,y,z); 0, 0, 0, 1];
% invRzA = @(angle,x,y,z) [invRz(angle), t(x,y,z); 0, 0, 0, 1];


%% Denavit Hartenberg
syms a alpha d theta

% T (i-1) --> i   =  Dx(a-1)*Rx(alpha-1)*Dz(d)*Rz(theta) 
transNext = @(a, alpha, d, theta) RxA(0,a,0,0)*RxA(alpha,0,0,0)*RzA(0,0,0,d)*RzA(theta,0,0,0);

% transPrev = @(a, alpha, d, theta) invRxA(0,a,0,0)*invRxA(alpha,0,0,0)*invRzA(0,0,0,d)*invRzA(theta,0,0,0);

%% DH Table to Transforms
% [a(i-1) alpha(i-1) d(i) theta(i)]

syms d1 th1 d2 th2 a2 d3 th3 a3 d4 th4 d5 th5 d6 

DHtable =   [0 0 d1 th1; ...
            0 pi/2 d2 th2; ...
            a2 0 d3 th3; ...
            a3 0 d4 th4; ...
            0 pi/2 d5 th5;
            0 0 d6 0];
        
final = eye(4);
        
f_final = length(DHtable);

frame_start = 1;
frame_stop = 4;


for row = frame_start:1:frame_stop
    
    a = DHtable(row,1);
    alpha = DHtable(row,2);
    d = DHtable(row,3);
    theta = DHtable(row,4);
    
    fprintf("Matrix: %d to %d",row-1,row)
    tmpMat(1:4,1:4) = transNext(a, alpha, d, theta)
    final = final*transNext(a, alpha, d, theta);
end

finalRot = simplify(final((1:3),(1:3)));
finalTrans = simplify(final((1:3),4))

%{
d2*sin(th1) - a3*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + d3*sin(th1) + d4*sin(th1) + a2*cos(th1)*cos(th2)
a3*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) + d2*cos(th1) + d3*cos(th1) + d4*cos(th1) - a2*cos(th2)*sin(th1)
                                                                                    d1 + a3*sin(th2 + th3) + a2*sin(th2)-d5
% chucked -d5 on top of calculated eq
%}

%% Plot
% theta1 = 0;
% theta2 = 0;
% theta3 = 0;
% 
% %assume wrist facing vertically down
% minang = 15;
% maxang = 75;
% 
% step = 5;
% val = (maxang-minang)/step;
% 
% x = zeros(val*val*val);
% y = zeros(val*val*val);
% z = zeros(val*val*val);
% index = 1;
% 
% d5 = 0;
% d4 = 0;
% d3 = 0;
% d2 = 0;
% 
% d1 = 0.05;
% a2 = 0.6;
% a3 = 0.5;
% for i = minang:step:maxang
%     for j = minang:step:maxang
%         for k = minang:step:maxang
%             x(index) = d2*sind(i) - a3*(cosd(i)*sind(j)*sind(-k) - cosd(i)*cosd(j)*cosd(-k)) + d3*sind(i) + d4*sind(i) + a2*cosd(i)*cosd(j);
%             y(index) = a2*cosd(j)*sind(i) - d2*cosd(i) - d3*cosd(i) - d4*cosd(i) - a3*(sind(i)*sind(j)*sind(-k)-cosd(j)*cosd(-k)*sind(i));
%             z(index) = d1 + a3*sind(j-k) + a2*sind(-j);
%             index = index + 1;
%         end
%     end
% end


%% Inverse
eqn1 = x == d2*sin(th1) - a3*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + d3*sin(th1) + d4*sin(th1) + a2*cos(th1)*cos(th2);
eqn2 = y == a3*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) + d2*cos(th1) + d3*cos(th1) + d4*cos(th1) - a2*cos(th2)*sin(th1);
eqn3 = z == d1 + a3*sin(th2 + th3) + a2*sin(th2)-d5;

eqm1 = subs(eqn1,{d2,d3,d4},{0,0,0})
eqm2 = subs(eqn2,{d2,d3,d4},{0,0,0})
eqm3 = subs(eqn3,{d2,d3,d4},{0,0,0})


% solve(eq1, eq2, eq3,[th1,th2,th3])
% [A,B] = equationsToMatrix([eqn1, eqn2, eqn3], [th1, th2, th3]);