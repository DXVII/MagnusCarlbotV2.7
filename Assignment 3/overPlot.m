clc
clear all
syms t
x_polys = [0.3180, -0.0116 * t^3 + 0.0520 * t^2 + 0.3180, 0.4740];
y_polys = [0.3180; 0.3180; 0.3180];
z_polys = [- 0.0111 * t^3 + 0.0500 * t^2; 0.1500; 0.0111 * t^3 - 0.0500 * t^2 + 0.1500];

% Intitial XZ plot
% Plot stem


time_limit = 9;
resolution2 = 10;

time = linspace(0, time_limit, resolution2);
t_seg1 = fix(resolution2 / 3);
t_seg2 = fix(2 * resolution2 / 3);



n = 1;
for i = 1:t_seg1
    fx1(n) = subs(x_polys(1), t, time(i));
    fy1(n) = subs(y_polys(1), t, time(i));
    fz1(n) = subs(z_polys(1), t, time(i));
    n = n + 1;
end

n = 1;
for i = 1:t_seg1
    fx2(n) = subs(x_polys(2), t, time(i));
    fy2(n) = subs(y_polys(2), t, time(i));
    fz2(n) = subs(z_polys(2), t, time(i));
    n = n + 1;
end

n = 1;
for i = 1:resolution2 - (2 * t_seg1)
    fx3(n) = subs(x_polys(3), t, time(i));
    fy3(n) = subs(y_polys(3), t, time(i));
    fz3(n) = subs(z_polys(3), t, time(i));
    n = n + 1;
end

x_vals = [fx1, fx2, fx3];
y_vals = [fy1, fy2, fy3];
z_vals = [fz1, fz2, fz3];

plot3(x_vals, y_vals, z_vals)
title("Traj")
xlabel("x axis")
zlabel("z axis")
ylabel("y axis")
axis padded;
grid on

x_start = 0;
y_start = 0;

d1 = 0.05; % shoulder
a2 = 0.4; % bicep
a3 = 0.30; % forearm
dE = 0.15; % hand length

th1_in = 45;
th2_in = 45;
th3_in = -20;

plot([0 0], [0 0.05]);
hold on;
[L1_start, L1_end, L2_start, L2_end, EE_start, EE_end] = generate_linkPoints(x_start, y_start, th1_in, th2_in, th3_in, d1, a2, a3, dE);
% Plot Link1
plot([L1_start(1) L1_end(1)], [L1_start(3) L1_end(3)], 'color', "r", 'linewidth', 2);
hold on;
% Plot Link2
plot([L2_start(1) L2_end(1)], [L2_start(3) L2_end(3)], 'color', "g", 'linewidth', 2);
hold on;
% Plot Link3
plot([EE_start(1) EE_end(1)], [EE_start(3) EE_end(3)], 'color', "b", 'linewidth', 2);
hold on;

[a,b,c] = invKin(0.3923, -0.0077, 0.3096, d1, a2, a3, dE)
[L1_start, L1_end, L2_start, L2_end, EE_start, EE_end] = generate_linkPoints(x_start, y_start, a, b, c, d1, a2, a3, dE);
% Plot Link1
plot([L1_start(1) L1_end(1)], [L1_start(3) L1_end(3)], 'color', "r", 'linewidth', 2);
hold on;
% Plot Link2
plot([L2_start(1) L2_end(1)], [L2_start(3) L2_end(3)], 'color', "g", 'linewidth', 2);
hold on;
% Plot Link3
plot([EE_start(1) EE_end(1)], [EE_start(3) EE_end(3)], 'color', "b", 'linewidth', 2);


%% Functions  -----------------------------------------------------------------------------------------------------------------------------------

function [L1_start_out, L1_end_out, L2_start_out, L2_end_out, EE_start_out, EE_end_out] = generate_linkPoints(x_start, y_start, th1_in, th2_in, th3_in, d1_in, a2_in, a3_in, dE_in)
    syms th1 th2 th3 d1 a2 a3 dE
    L1_start = [x_start; y_start; d1];
    L1_end = [a2 * cosd(th1) * cosd(th2); -a2 * cosd(th2) * sind(th1); a2 * sind(th2) + d1];

    L2_start = L1_end;
    L2_end = [a2 * cosd(th1) * cosd(th2) + a3 * cosd(th1) * cosd(th2 + th3); -a2 * cosd(th2) * sind(th1) + a3 * sind(th1) * cosd(th2 + th3); a2 * sind(th2) + a3 * sind(th2 + th3) + d1];

    EE_start = L2_end;
    EE_end = L2_end + [0; 0; -dE];

    L1_start_out = subs(L1_start, [th1, th2, th3, d1, a2, a3, dE], [th1_in, th2_in, th3_in, d1_in, a2_in, a3_in, dE_in]);
    L1_end_out = subs(L1_end, [th1, th2, th3, d1, a2, a3, dE], [th1_in, th2_in, th3_in, d1_in, a2_in, a3_in, dE_in]);
    L2_start_out = subs(L2_start, [th1, th2, th3, d1, a2, a3, dE], [th1_in, th2_in, th3_in, d1_in, a2_in, a3_in, dE_in]);
    L2_end_out = subs(L2_end, [th1, th2, th3, d1, a2, a3, dE], [th1_in, th2_in, th3_in, d1_in, a2_in, a3_in, dE_in]);
    EE_start_out = subs(EE_start, [th1, th2, th3, d1, a2, a3, dE], [th1_in, th2_in, th3_in, d1_in, a2_in, a3_in, dE_in]);
    EE_end_out = subs(EE_end, [th1, th2, th3, d1, a2, a3, dE], [th1_in, th2_in, th3_in, d1_in, a2_in, a3_in, dE_in]);

end

function [th1, th2, th3] = invKin(x, y, z, d1, a2, a3, dE)
    th1 = atan2d(y, x);
    H = sqrt(x^2 + y^2 + (z + dE - d1)^2);

    % th2 = atan2d((z + dE - d1), sqrt(x^2 + y^2)) + acosd((a2^2 + H^2 - a3^2) / (2 * a2 * H));
    th2 = sind((z + dE - d1)/sqrt(x^2 + y^2))/cosd((z + dE - d1)/sqrt(x^2 + y^2)) + acosd((a2^2 + H^2 - a3^2) / (2 * a2 * H));
    th3 = acosd((a2^2 + a3^3 - H^2) / (2 * a2 * a3)) - 180;

    % th4 = -th2 - th3;
    % th5 = th1;
end
