%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% -- Traj -------------------------------------------------------------------------------------------
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parameters for plotting trajectory
time_limit = 9;
startCoord = [0.1188,-0.1313, 0.016];
endCoord = [0.2687, -0.1313, 0.016];
height = 5;

% Finding coefficients
[x_coeffs, y_coeffs, z_coeffs] = findCoeffs(time_limit, startCoord, endCoord, height)
x_polys = polyfuncs(x_coeffs)
y_polys = polyfuncs(y_coeffs)
z_polys = polyfuncs(z_coeffs)

% Generating plotting vectors
PATHS = 3;
resolution = 100;
time = linspace(0, time_limit, resolution);

t_seg1 = fix(resolution / 3) + 1;
t_seg2 = fix(2 * resolution / 3) + 1;

% x_vals = zeros(1, resolution);
% y_vals = zeros(1, resolution);
% z_vals = zeros(1, resolution);

syms t
n = 1;

for i = 1:t_seg1
    fx1(n) = subs(x_polys(1), t, time(i));
    fy1(n) = subs(y_polys(1), t, time(i));
    fz1(n) = subs(z_polys(1), t, time(i));
    n = n + 1;
end

n = 1;

for i = t_seg1 + 1:t_seg2
    fx2(n) = subs(x_polys(2), t, time(i));
    fy2(n) = subs(y_polys(2), t, time(i));
    fz2(n) = subs(z_polys(2), t, time(i));
    n = n + 1;
end

n = 1;

for i = t_seg2 + 1:resolution
    fx3(n) = subs(x_polys(3), t, time(i));
    fy3(n) = subs(y_polys(3), t, time(i));
    fz3(n) = subs(z_polys(3), t, time(i));
    n = n + 1;
end

x_vals = [fx1, fx2, fx3];
y_vals = [fy1, fy2, fy3];
z_vals = [fz1, fz2, fz3];

% f_choice

subplot(2, 3, 1)
plot(time, x_vals)
title("x over time")
xlabel("time")
ylabel("x coord")
grid on
axis padded

subplot(2, 3, 2)
plot(time, y_vals)
title("y over time")
xlabel("time")
ylabel("y coord")
grid on
axis padded

subplot(2, 3, 3)
plot(time, z_vals)
title("z over time")
xlabel("time")
ylabel("z coord")
grid on
axis padded

subplot(2, 3, 4)
plot(x_vals, y_vals)
title("top down view")
xlabel("x coord")
ylabel("y coord")
grid on
axis padded

subplot(2, 3, 5)
plot(x_vals, z_vals)
title("side (x) view")
xlabel("x coord")
ylabel("z coord")
grid on
axis padded

subplot(2, 3, 6)
plot3(x_vals, y_vals, z_vals)
title("3D plot")
xlabel("x coord")
ylabel("y coord")
zlabel("z coord")
grid on
axis padded

function polyMat = polyfuncs(a_coeffs)
    syms t
    tmp = flip(a_coeffs);
    % coefficients for poly2sym are in order [a0,a1,a2,a3]
    % poly2sym turns vector of coefficients into a polynomcal
    for i = 1:size(tmp, 2)
        polyMat(i, 1) = poly2sym(transpose(tmp(:, i)), t);
    end

end

function [x_coeffs, y_coeffs, z_coeffs] = findCoeffs(time_limit, startCoord, endCoord, height)
    x_i = startCoord(1);
    y_i = startCoord(2);
    z_i = startCoord(3);

    x_f = endCoord(1);
    y_f = endCoord(2);
    z_f = endCoord(3);

    % time_limit = 5;
    x_points = [x_i, x_i, x_f, x_f]
    y_points = [y_i, y_i, y_f, y_f]

    % height = 10;
    z_points = [z_i, z_i + height, z_f + height, z_f]

    % intermediary points are set to 0
    zeroVel = zeros(1, 4);

    x_coeffs = traj_3part(time_limit, x_points, zeroVel);
    y_coeffs = traj_3part(time_limit, y_points, zeroVel);
    z_coeffs = traj_3part(time_limit, z_points, zeroVel);

end

function abc_coeffs = traj_3part(t_lim, points, velos)
    XA = points(1);
    XB = points(2);
    XC = points(3);
    XD = points(4);

    VA = velos(1);
    VB = velos(2);
    VC = velos(3);
    VD = velos(4);

    ti = 0;
    tf = t_lim / 3;

    a0 = XA;
    a1 = 0;
    a2 = (-3*XA + 3*XB) / tf^2;
    a3 = (2*XA - 2*XB) / tf^3;

    b0 = XB;
    b1 = 0;
    b2 = (-3*XB + 3*XC) / tf^2;
    b3 = (2*XB - 2*XC) / tf^3;

    c0 = XC;
    c1 = 0;
    c2 = (-3*XC + 3*XD) / tf^2;
    c3 = (2*XC - 2*XD)/ tf^3;

    AB = [a0; a1; a2; a3];
    BC = [b0; b1; b2; b3];
    CD = [c0; c1; c2; c3];

    abc_coeffs = [AB, BC, CD];
end

% function abc_coeffs = traj_3part(t_lim, points, velos)
%     XA = points(1); 
%     XB = points(2);
%     XC = points(3);
%     XD = points(4);

%     VA = velos(1);
%     VB = velos(2);
%     VC = velos(3);
%     VD = velos(4);
    
%     ti = 0;
%     tf = t_lim / 3;
    
%     a0 = XA;     
%     a1 = 0;
%     a2 = (-3*XA + 3*XB - VB*tf) / tf^2;
%     a3 = (2*XA - 2*XB + VB*tf) / tf^3;
    
%     b0 = XB;
%     b1 = VB;
%     b2 = (-3*XB + 3*XC - 2*VB*tf - VC*tf) / tf^2;
%     b3 = (2*XB - 2*XC + VB*tf + VC*tf)  / tf^3;

%     c0 = XC;     
%     c1 = VC;
%     c2 = (-3*XC + 3*XD - 2*VC*tf) / tf^2;
%     c3 = (2*XC - 2*XD + VC*tf) / tf^3;

%     AB = [a0; a1; a2; a3];
%     BC = [b0; b1; b2; b3];
%     CD = [c0; c1; c2; c3];
    

%     abc_coeffs = [AB, BC, CD];
% end

% TODO:
% (tick) define XB and XC as a vert height about XA (xi) and XD (xf) by height
% () generate VB and VC terms
% %}
