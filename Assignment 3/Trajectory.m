%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% -- Traj -------------------------------------------------------------------------------------------
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parameters for plotting trajectory
time_limit = 5;
startCoord = [5, 5, 5];
endCoord = [10, 10, 5];
height = 5;


% Finding coefficients
[x_coeffs, y_coeffs, z_coeffs] = findCoeffs(time_limit, startCoord, endCoord, height);
x_polys = polyfuncs(x_coeffs);
y_polys = polyfuncs(y_coeffs);
z_polys = polyfuncs(z_coeffs);


% Generating plotting vectors
PATHS = 3;
resolution = 100;
time = linspace(0,time_limit,resolution);

t1 = resolution/3;
t2 = 2*resolution / 3;
x_vals = zeros(1, resolution);
y_vals = zeros(1, resolution);
z_vals = zeros(1, resolution);
f_choice = zeros(1, resolution);

syms t
for i=1:resolution
    if (i<=t1)
        fx = x_polys(1);
        fy = y_polys(1);
        fz = z_polys(1);
        f_choice(i) = 1;
    elseif (i > t1 && i<=t2)
        fx = x_polys(2);
        fy = y_polys(2);
        fz = z_polys(2);
        f_choice(i) = 2;
    elseif (i>t2)
        fx = x_polys(3);
        fy = y_polys(3);
        fz = z_polys(3);
        f_choice(i) = 3;
    else
        f_choice(i) = 4;
    end

    x_vals(i) = subs(fx, t, time(i));
    y_vals(i) = subs(fy, t, time(i));
    z_vals(i) = subs(fz, t, time(i));
    
end
subplot(2,3,1)
plot(time, x_vals)
title("x over time")
xlabel("time")
ylabel("x coord")

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
    zeroVel = zeros(1,4);

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
    t2 = 2 * t_lim / 3;
    t3 = t_lim;

    % p1 = 0; p2 = 0; p3 = 3; p4 = 3;
    % v1 = 0; v2 = 0; v3 = 0; v4 = 0;

    % traj_seg(t_i, t_f, p_i, p_f, v_i, v_f)
    
    AB = traj_seg(t0, t1, p1, p2, v1, v2);
    BC = traj_seg(t1, t2, p2, p3, v2, v3);
    CD = traj_seg(t2, t3, p3, p4, v3, v4);
    
    % AB = [AB0; AB1; AB2; AB3] = traj_seg(t0, t1, p1, p2, v1, v2);
    % BC = [BC0; BC1; BC2; BC3] = traj_seg(t1, t2, p2, p3, v2, v3);
    % CD = [CD0; CD1; CD2; CD3] = traj_seg(t2, t3, p3, p4, v3, v4);
    abc_coeffs = [AB, BC, CD];
end

function sols = traj_seg(t_i, t_f, p_i, p_f, v_i, v_f)
% function [m0, m1, m2, m3] = traj_seg(t_i, t_f, p_i, p_f, v_i, v_f)
    % syms t_i t_f
    % syms p_i p_f 
    % syms v_i v_f
    % syms secdiv

    syms t
    syms m0 m1 m2 m3 
    traj = m0 + m1*t + m2*t.^2 + m3*t.^3;
    velo = m1 + m2*t + m3*t.^2;


    % t_i = 0;         % for now
    % secdiv = 3;
    % tf = 5/secdiv;  % 

    f1 = p_i == subs(traj, t, t_i);
    f2 = p_f == subs(traj, t, t_f);
    f3 = v_i == subs(velo, t, t_i);
    f4 = v_f == subs(velo, t, t_f);

    eqns = [f1, f2, f3, f4];
    coefvars = [m0, m1, m2, m3];
    [A, b] = equationsToMatrix(eqns, coefvars);
    sols = A * b;

    % m0 = sols(1); 
    % m1 = sols(2); 
    % m2 = sols(3); 
    % m3 = sols(4);
end


% TODO:
% (tick) define XB and XC as a vert height about XA (xi) and XD (xf) by height
% () generate VB and VC terms
% %}



% %% ---------------------------------------------------------------------------------------------------
% %% -- Pose -------------------------------------------------------------------------------------------
% %% ---------------------------------------------------------------------------------------------------
function pose()
    %% Creating Rotations, Translations, Augmented Matrix
    syms x y z angle

    % Translation
    t = @(x, y, z) [x; y; z];

    % Rotation
    Rx = @(angle)[1, 0, 0; 0, cos(angle), sin(angle); 0, -sin(angle), cos(angle)];
    Rz = @(angle) [cos(angle), sin(angle), 0; -sin(angle), cos(angle), 0; 0, 0, 1];

    % Augment
    RxA = @(angle, x, y, z) [Rx(angle), t(x, y, z); 0, 0, 0, 1];
    RzA = @(angle, x, y, z) [Rz(angle), t(x, y, z); 0, 0, 0, 1];

    %% Part 1: Denavit Hartenberg Method
    syms a alpha d theta

    % Transform Chain function:  T (i-1) --> i   =  Dx(a-1)*Rx(alpha-1)*Dz(d)*Rz(theta)
    transNext = @(a, alpha, d, theta) RxA(0, a, 0, 0) * RxA(alpha, 0, 0, 0) * RzA(0, 0, 0, d) * RzA(theta, 0, 0, 0);

    % DH Table to Transforms
    % [a(i-1) alpha(i-1) d(i) theta(i)]
    syms d1 th1 d2 th2 a2 d3 th3 a3 d4 th4 d5 th5 d6
    DHtable = [0 0 d1 th1; ... % frame 0-1: XY shoulder swivel
                0 pi / 2 d2 th2; ... % frame 1-2: XZ bicep elevation
                a2 0 d3 th3; ... % frame 2-3: XZ elbow descent
                a3 0 d4 th4; ... % frame 3-4: XZ vertical wrist (constraint: point to ground)
                0 pi / 2 d5 th5; ... % frame 4-5: XY wrist torsional twist (constraint: point foward)
                0 0 d6 0]; % frame 5-6: Wrist to Gripper distance

    %% Generate Poses
    final = eye(4); % being DH chain

    f_final = length(DHtable); % code tings: simplify end of array

    % frame 1 to 4 chosen due to relational constrains of frames 4 and onwards (discussed in report)
    frame_start = 1; % code tings: start of transform chain
    frame_stop = 6; % code tings: end of transform chain

    translations = sym([0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0]);

    for row = frame_start:1:frame_stop
        a = DHtable(row, 1);
        alpha = DHtable(row, 2);
        d = DHtable(row, 3);
        theta = DHtable(row, 4);

        % fprintf("Matrix: %d to %d", row - 1, row);
        tmpMat(1:4, 1:4) = transNext(a, alpha, d, theta);
        positions(:, row) = tmpMat(1:3, 4);
        % summation of positions from "shoulder joint"
        rotStart = (row * 4 - 3);
        rotEnd = (row * 4 - 1);
        rotations(rotStart:rotEnd, 1:3) = tmpMat(1:3, 1:3);
        % invrotations(rotStart:rotEnd, 1:3) = transpose(tmpMat(1:3, 1:3));

    end

    %% Rotations
    % rotations     --> R01  --> getRotMat(1, rotations)      --> R0'1'
    % invrotations  --> R_10 --> getRotMat(1, invrotations)   --> R_'1'0
    R01 = getRotMat(1, rotations);
    R12 = getRotMat(2, rotations);
    R23 = getRotMat(3, rotations);
    R34 = getRotMat(4, rotations);
    R45 = getRotMat(5, rotations);
    R56 = getRotMat(6, rotations);

    R02 = R01 * R12;
    R03 = R01 * R12 * R23;
    R04 = R01 * R12 * R23 * R34;
    R05 = R01 * R12 * R23 * R34 * R45;
    R06 = R05;


    %% Stuff
    % only need R45

    R0F = R45;
    theta_f = 2*acosd(0.5*sqrt(1 + R0F(1,1) + R0F(2,2) + R0F(3,3)))

    k_hat = (1 / (2 * sind(theta_f))) * [(R0F(3,2)-R0F(2,3)); (R0F(1,3)-R0F(3,1)); (R0F(2,1)-R0F(1,2))]
    kx = k_hat(1);
    ky = k_hat(2);
    kz = k_hat(3);




syms p0 p1 p2 p3 t
f_theta = p0 + p1.*t + p2.*t.^2 + p3.*t.^3  % begins at 0  =>  theta_f at the end of time given
e1 = kx * sind(theta_f/2);
e2 = ky * sind(theta_f/2);
e3 = kz * sind(theta_f/2);
e4 = cosd(theta_f/2);



Rit = [ (1-2*e2^2-2*e3^2)   ,   2*(e1*e2-e3*e4)     ,   2*(e1*e3+e2*e4) ;...
        2*(e1*e2-e3*e4)     ,   (1-2*e1^2-2*e3^2)   ,   2*(e2*e3-e1*e4) ;...
        2*(e1*e3+e2*e4)     ,   2*(e2*e3-e1*e4)     ,   (1-2*e1^2-2*e2^2)
      ];

R0t = R04 * Rit;


%{
    a0 = XA
    a1 = XA + 4.4444 * XB + 4.6296 * XC
    a2 = 4.6296 * VB + XC + 1.6667 * XD
    a3 = 37.0370 * VB + XC + 3.3333 * XD
    b0 = VB + 14.4444 * VC
    b1 = VB + 30 * VC
    b2 = XB
    b3 = 4.3333 * XB + 8.3333 * XC
    c0 = 8.3333 * VB + XD
    c1 = 33.3333 * VB + XD
    c2 = 7.6667 * VC
    c3 = 11 * VC



    theta_f = 114.5916 * acos(0.5000 * (cos(th5) + 1)^(1/2))
    th5 = th1


    k_hat =
    k_x = -(0.5000 * (cos(th5) + 1)) / sin(2 * acos(0.5000 * (cos(th5) + 1)^(1/2)))
    k_y = -(0.5000 * sin(th5)) / sin(2 * acos(0.5000 * (cos(th5) + 1)^(1/2)))
    k_z = -(0.5000 * sin(th5)) / sin(2 * acos(0.5000 * (cos(th5) + 1)^(1/2)))

%}
end

function RotMat = getRotMat(num, rotations)
    rotStart = (num * 4 - 3);
    rotEnd = (num * 4 - 1);
    RotMat = rotations(rotStart:rotEnd, 1:3);
end
