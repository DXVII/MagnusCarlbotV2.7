% Group 27
% Parameters for plotting trajectory

square_width = 3.9/100;
edge_buffer = 2.1/100;
origin_offset = 20/100;
grab_height = 3.5/100;
avoid_height = 15/100;

d1 = 0.05; % shoulder
a2 = 0.4; % bicep
a3 = 0.30; % forearm
dE = 0.15; % hand length

th1_start = 0;
th2_start = 0;
th3_start = 0;

time_limit = 9;
startCoord = [0.318, 0.318, 0];
endCoord = [0.474, 0.318, 0];
height = 0.15;
resolution = 100;

[x_polys, y_polys, z_polys] = plotViews(time_limit, startCoord, endCoord, height, resolution);


% Forward Kinematics the links
% [L1_start, L1_end, L2_start, L2_end, EE_start, EE_end] = generate_linkPoints(th1_start, th2_start, th3_start, d1, a2, a3, dE);
% Plot stem
figure()
plot([0 0], [0 d1], 'color', "c", 'linewidth', 2);
hold on;
% % Plot Link1
% plot([L1_start(1) L1_end(1)], [L1_start(3) L1_end(3)]);
% hold on;
% % Plot Link2
% plot([L2_start(1) L2_end(1)], [L2_start(3) L2_end(3)]);
% hold on;
% % Plot Link3
% plot([EE_start(1) EE_end(1)], [EE_start(3) EE_end(3)]);
% hold on;



resolution2 = 10;
time = linspace(0, time_limit, resolution2);
t_seg1 = fix(resolution2/3);
t_seg2 = fix(2 * resolution2/3);

syms t

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

th1 = zeros(1,length(x_vals));
th2 = zeros(1,length(x_vals));
th3 = zeros(1,length(x_vals));

% generating inv kinematic angles
for i=1:length(x_vals)
    [th1_i,th2_i,th3_i] = invKin(x_vals(i), y_vals(i), z_vals(i), d1, a2, a3, dE);
    th1(i) = th1_i;
    th2(i) = th2_i;
    th3(i) = th3_i;
end

% plotting overlay of links
for i = 1:length(th1)
    % inverse kinematics links 
    [L1_start, L1_end, L2_start, L2_end, EE_start, EE_end] = generate_linkPoints(th1(i), th2(i), th3(i), d1, a2, a3, dE);
    % Plot Link1
    plot([L1_start(1) L1_end(1)], [L1_start(3) L1_end(3)],'color',"r",'linewidth',2);
    hold on;
    % Plot Link2
    plot([L2_start(1) L2_end(1)], [L2_start(3) L2_end(3)],'color',"g",'linewidth',2);
    hold on;
    % Plot Link3
    plot([EE_start(1) EE_end(1)], [EE_start(3) EE_end(3)],'color',"b",'linewidth',2);
    hold on;

    
end

plot(x_vals+0.038,z_vals+0.05,'color',"m",'linewidth',1);
hold on;

plot([0.338 0.338 + square_width * 8], [0.05, 0.05], 'k')
hold on

offset_x = 0.338;
for i = 1:8
    plot([square_width * (i - 1) + offset_x square_width * (i - 1) + offset_x], [0.05 0.04], 'k')      % Chessboard tile
    hold on;
end

title("XZ Trajectory")
xlabel("x axis (m)")
ylabel("z axis (m)")
legend("stem","Link1","Link2","link To EE")
axis padded;
grid on




%% Functions  -----------------------------------------------------------------------------------------------------------------------------------

function [L1_start_out, L1_end_out, L2_start_out, L2_end_out, EE_start_out, EE_end_out] = generate_linkPoints(th1_in, th2_in, th3_in, d1_in, a2_in, a3_in, dE_in)
    syms th1 th2 th3 d1 a2 a3 dE

    x_start = 0;
    y_start = 0;
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
    th1 = atan2d(y,x);
    H = sqrt(x^2 + y^2 + (z+dE - d1)^2);

    th2 = atan2d((z + dE - d1), sqrt(x^2 + y^2)) + acosd((a2^2 + H^2 - a3^2) / (2 * a2 * H));
    th3 = acosd((a2^2 + a3^3 - H^2) / (2 * a2 * a3)) - 180;

    % th4 = -th2 - th3;
    % th5 = th1;
end

function [x_polys, y_polys, z_polys] = plotViews(time_limit, startCoord, endCoord, height, resolution)
    % Finding coefficients
    [x_coeffs, y_coeffs, z_coeffs] = findCoeffs(time_limit, startCoord, endCoord, height)
    x_polys = polyfuncs(x_coeffs);
    y_polys = polyfuncs(y_coeffs);
    z_polys = polyfuncs(z_coeffs);


    % Generating plotting vectors
    
    time = linspace(0,time_limit,resolution);

    t_seg1 = fix(resolution/3);
    t_seg2 = fix(2*resolution/3);

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

    subplot(2,3,1)
    plot(time, x_vals)
    title("X over Time")
    xlabel("time (s)")
    ylabel("x coord (m)")
    grid on
    axis padded

    subplot(2,3,2)
    plot(time, y_vals)
    title("Y over Time")
    xlabel("time (s)")
    ylabel("y coord (m)")
    grid on
    axis padded

    subplot(2,3,3)
    plot(time, z_vals)
    title("Z over Time")
    xlabel("time (s)")
    ylabel("z coord (m)")
    grid on
    axis padded

    % Chess squares
    offset = 0.3;
    square_width = 3.9/100;

    subplot(2,3,4)
    plot(x_vals, y_vals)
    hold on
    for i = 1:8
        x1 = square_width*(i-1) + offset;
        x2 = square_width*(i) + offset;
        for j = 1:8
            y1 = square_width * (j - 1) + offset;
            y2 = square_width * (j) + offset;
            plot([x1 x1 x2 x2 x1],[y1 y2 y2 y1 y1],'k')
            hold on;
        end
    end
    title("Top-Down (XY) View")
    xlabel("x coord (m)")
    ylabel("y coord (m)")
    grid on
    axis padded

    subplot(2,3,5)
    plot(x_vals, z_vals)
    hold on
    
    plot([0.3 0.3+square_width*8],[0,0],'k')
    hold on

    for i = 1:8
        plot([square_width*(i-1) + offset, square_width*(i-1) + offset], [0 -0.005], 'k')  
        hold on;
    end
    
    title("Side (XZ) View")
    xlabel("x coord (m)")
    ylabel("z coord (m)")
    grid on
    axis padded

    subplot(2,3,6)
    plot3(x_vals, y_vals, z_vals)
    hold on
    for i = 1:8
        x1 = square_width * (i - 1) + offset;
        x2 = square_width * (i) + offset;

        for j = 1:8
            y1 = square_width * (j - 1) + offset;
            y2 = square_width * (j) + offset;
            plot3([x1 x1 x2 x2 x1], [y1 y2 y2 y1 y1],[0 0 0 0 0], 'k')
            hold on;
        end

    end

    
    title("3D plot")
    xlabel("x coord (m)")
    ylabel("y coord (m)")
    zlabel("z coord (m)")
    grid on
    axis padded
end

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
    x_points = [x_i, x_i, x_f, x_f];
    y_points = [y_i, y_i, y_f, y_f];

    % height = 10;
    z_points = [z_i, z_i + height, z_f + height, z_f];

    % intermediary points are set to 0
    zeroVel = zeros(1,4);
    % zeroVel = [0,0.01,0.01,0];

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

    t0 = 0;
    t1 = t_lim/3;
        
    AB = traj_seg(t0, t1, p1, p2, v1, v2);
    BC = traj_seg(t0, t1, p2, p3, v2, v3);
    CD = traj_seg(t0, t1, p3, p4, v3, v4);
    
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

end


