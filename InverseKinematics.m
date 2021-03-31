%{
From Prev
d2*sin(th1) - a3*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + d3*sin(th1) + d4*sin(th1) + a2*cos(th1)*cos(th2)
a3*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) + d2*cos(th1) + d3*cos(th1) + d4*cos(th1) - a2*cos(th2)*sin(th1)
                                                                                         d1 + a3*sin(th2 + th3) + a2*sin(th2) -(d5+d6)
% chucked -(d5+d6) on top of calculated eq
%}

%% Inverse
syms d1 th1 d2 th2 a2 d3 th3 a3 d4 th4 d5 th5 d6
fx = d2*sin(th1) - a3*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + d3*sin(th1) + d4*sin(th1) + a2*cos(th1)*cos(th2);
fy = a3*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) + d2*cos(th1) + d3*cos(th1) + d4*cos(th1) - a2*cos(th2)*sin(th1);
fz = d1 + a3*sin(th2 + th3) + a2*sin(th2)-(d5+d6);

%% Linearise
% To find valid equilibria

% equilibrium points, (to be found)
eq_th1 = pi/4;
eq_th2 = pi/4;
eq_th3 = pi/4;
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

[coeffs, Coords] = equationsToMatrix([lin_x, lin_y, lin_z], [th1, th2, th3]);

Thetas = inv(coeffs)*Coords;
th1 = Thetas(1)
th2 = Thetas(2)
th3 = Thetas(3)

