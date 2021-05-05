%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% -- Traj -------------------------------------------------------------------------------------------
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parameters for plotting trajectory
time_limit = 9;
startCoord = [0.1188,-0.1313, 0.016];
endCoord = [0.2687, -0.1313, 0.016];
height = 0.15;


% Finding coefficients
[x_coeffs, y_coeffs, z_coeffs] = findCoeffs(time_limit, startCoord, endCoord, height)
x_polys = polyfuncs(x_coeffs);
y_polys = polyfuncs(y_coeffs);
z_polys = polyfuncs(z_coeffs);


% Generating plotting vectors
PATHS = 3;
resolution = 100;
time = linspace(0,time_limit,resolution);

t_seg1 = fix(resolution/3);
t_seg2 = fix(2*resolution/3);


% x_vals = zeros(1, resolution);
% y_vals = zeros(1, resolution);
% z_vals = zeros(1, resolution);

syms t
n=1;
for i=1:t_seg1
    fx1(n) = subs(x_polys(1), t, time(i));
    fy1(n) = subs(y_polys(1), t, time(i));
    fz1(n) = subs(z_polys(1), t, time(i));
    n=n+1;    
end
n=1;
for i = 1:t_seg1
    fx2(n) = subs(x_polys(2), t, time(i));
    fy2(n) = subs(y_polys(2), t, time(i));
    fz2(n) = subs(z_polys(2), t, time(i));
    n=n+1;    
end
n=1;
for i = 1:resolution-(2*t_seg1)
    fx3(n) = subs(x_polys(3), t, time(i));
    fy3(n) = subs(y_polys(3), t, time(i));
    fz3(n) = subs(z_polys(3), t, time(i));
    n=n+1;    
end


x_vals = [fx1, fx2, fx3];
y_vals = [fy1, fy2, fy3];
z_vals = [fz1, fz2, fz3];
    

% f_choice

subplot(2,3,1)
plot(time, x_vals)
title("x over time")
xlabel("time")
ylabel("x coord")
grid on
axis padded

subplot(2,3,2)
plot(time, y_vals)
title("y over time")
xlabel("time")
ylabel("y coord")
grid on
axis padded

subplot(2,3,3)
plot(time, z_vals)
title("z over time")
xlabel("time")
ylabel("z coord")
grid on
axis padded

subplot(2,3,4)
plot(x_vals, y_vals)
title("top down view")
xlabel("x coord")
ylabel("y coord")
grid on
axis padded

subplot(2,3,5)
plot(x_vals, z_vals)
title("side (x) view")
xlabel("x coord")
ylabel("z coord")
grid on
axis padded

subplot(2,3,6)
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
    for i = 1:size(tmp,2)
        polyMat(i,1) = poly2sym(transpose(tmp(:,i)),t);
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
    % zeroVel = zeros(1,4);
    zeroVel = [0,0.001,0.001,0];

    x_coeffs = traj_3part(time_limit, x_points, zeroVel);
    y_coeffs = traj_3part(time_limit, y_points, zeroVel);
    z_coeffs = traj_3part(time_limit, z_points, zeroVel);

end

function abc_coeffs = traj_3part(t_lim, points, velos)  
    p1 = points(1);
    p2 = points(2);
    p3 = points(3);
    p4 = points(4);
    
    v1 = velos(1);
    v2 = velos(2);
    v3 = velos(3);
    v4 = velos(4);

    % t_lim = 5;

    t0 = 0;
    t1 = t_lim / 3;
    
    % p1 = 0; p2 = 0; p3 = 3; p4 = 3;
    % v1 = 0; v2 = 0; v3 = 0; v4 = 0;

    % traj_seg(t_i, t_f, p_i, p_f, v_i, v_f)
    
    AB = traj_seg(t0, t1, p1, p2, v1, v2);
    BC = traj_seg(t0, t1, p2, p3, v2, v3);
    CD = traj_seg(t0, t1, p3, p4, v3, v4);
    
    % AB = [AB0; AB1; AB2; AB3] = traj_seg(t0, t1, p1, p2, v1, v2);
    % BC = [BC0; BC1; BC2; BC3] = traj_seg(t1, t2, p2, p3, v2, v3);
    % CD = [CD0; CD1; CD2; CD3] = traj_seg(t2, t3, p3, p4, v3, v4);
    abc_coeffs = [AB, BC, CD];
end

function sols = traj_seg(t_i, t_f, p_i, p_f, v_i, v_f)
    syms t
    syms m0 m1 m2 m3 
    traj = m0 + m1*t + m2*t^2 + m3*t^3;
    velo = m1 + 2*m2*t + 3*m3*t^2;

    f1 = p_i == subs(traj, t, t_i);
    f2 = p_f == subs(traj, t, t_f);
    f3 = v_i == subs(velo, t, t_i);
    f4 = v_f == subs(velo, t, t_f);

    eqns = [f1, f2, f3, f4];
    coefvars = [m0, m1, m2, m3];
    [A, b] = equationsToMatrix(eqns, coefvars);
    sols = linsolve(A , b);

    % m0 = sols(1); 
    % m1 = sols(2); 
    % m2 = sols(3); 
    % m3 = sols(4);
end



