% ChessBot_Skeleton_Code.mlx
% --------------------------
% Licenting Information: You are free to use or extend this project
% for educational purposes provided that (1) you do not distribute or
% publish solutions, (2) you retain this notice, and (3) you provide
% clear attribution to the University of Melbourne, Department of
% Mechanical Engineering.
%
% Attribution Information: The ChessBot project was developed at the
% University of Melbourne. The core project was primarily developed
% by Professor Denny Oetomo (doetomo@unimelb.edu.au). The ChessBot
% Skeleton Code was developed by Nathan Batham
% (nathan.batham@unimelb.edu.au)

%% How To Use This Skeleton Code

% Set COM Port          - Under the "Establish Serial Connection" section, select
%               the appropriate port format and identifier. This should be the
%               same as what is shown in the Arduino IDE software under 'Tools ->
%               Port'.

% Executing Section     - To execute a single section this Live Script, click into
%               the section, so that it is highlighted on the left hand side. Then,
%               press ctrl+enter (or cmd+enter on mac).

% Sample Code           - The functions provided have been designed as a guide for
%               basic functionality only. It is advised to expand them and dig into
%               the code to truely understand the the motor capabilities. It is also
%               recommended to remove the sections in this file labelled "Sample Code"
%               once you have become familiar with how each operation works.

% Position Feedback     - The feedback from the provided motors is only betwen -pi/2
%               and +pi/2. As such, it is NOT ADVISED to use gearboxes or any form
%               of mechanical reduction. This will also limit your reachable workspace.
%               In the Arduino code, the vector "motorOffset" contains the offset used
%               to set the range of motion so that the centre is at 0. This can be
%               modified if required to increase motion in one direction or the other,
%               which may help with some reachable workspace and home position issues.

% Closing Serial        - IMPORTANT: The serial connection MUST be closed before either
%               a) disconnecting the Arduino, or b) clearing the serial object.
%               Failure to do so may result in the COM port becoming inaccessible.
%               In the instance that this does occur, a full system restart will
%               be required. Please see fclose() line at the end of this file and
%               execute whenever necessary.

% BUG REPORTING         - If you believe there may be a bug in the code, please report
%               it using the subject discussion board. Any revisions will be uploaded
%               to CANVAS and all students will be notified.

%% Establish Serial Connection
% Open the port to which the Arduino is connected and create a serial object.

% @ port                - COM port address of the Arduino. This will change depending on which USB port
%                   the arduino is connected to, and the exact structure of the address will vary between
%                   operating systems

% @ baudrate            - BaudRate used between MATLAB and Arduino, which is limited to a max of
%                   230400 by MATLAB.

% @ numID               - Number of detected motors.

% @ ID                  - Vector containing ID of each detected motor.

% @ establishSerial()   - A helper function that creates and returns a serial object while also performing
%                   a handshake with the Arduino to confim connection.

% Specify COM Port
% port = '/dev/cu.usbmodem14141';                  % MacOS Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS
port = 'COM15'; % Windows Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS

% Setup Serial Connection
baudrate = 230400;
s = establishSerial(port, baudrate);

% Read Connected Motor IDs
[numID, ID] = getMotorIDs(s)

% Set Change Motor ID flag - CHANGE TO ADJUST MOTOR ID
changeMotorID = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% NOTE: Remember to fclose(s) before disconnecting your Arduino!!! %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Sample Code for Reading Motor Feedback
% Read motor position feedback
[motorFB, eFB] = readFB(s, numID); % radian/s
motorFB % Display motor feedback
% will need both task space and joint space control
% if you using position control (it uses internal PID), velocity control
% n/a

%% Sample Code for Changing To Position or Velocity Control Mode
% Set motor EPROM to position control mode
setControlMode(s, "position");
% Set motor EPROM to velocity control mode
setControlMode(s, "velocity");

%% Sample Code for Sending Position Commands - Motors Must Be In Position Control Mode
% Send basic speed up and slow down trajectory to motors
for v = [0:0.001:1.8762 1.8762:-0.001:-1.8762 -1.8762:0.001:0]

    motorPos = zeros(1, numID) + v;

    % To control each motor individually change the above line to be in a format like this:
    % motorVel = [q0 q1 q2 q3 q4 q5];

    sendJointPos(s, motorPos, numID);
end

%% Sample Code for Sending Velocity Commands - Motors Must Be In Velocity Control Mode
% Send basic speed up and slow down trajectory to motors
for v = [0:0.01:pi pi:-0.01:-pi -pi:0.01:0]
    motorVel = zeros(1, numID) + v;

    % To control each motor individually change the above line to be in a format like this:
    % motorVel = [q0 q1 q2 q3 q4 q5];
    sendJointVel(s, motorVel, numID);

    % Delay
    for i = 1:100
    end

end

%% Sample Code for Setting Motor PID - Only Valid for Position Control Mode
% Set Motor internal PID controller. Gains must be integers.
tmpID = 1;
Kp = 10;
Ki = 0;
Kd = 1;
setPID(s, tmpID, Kp, Ki, Kd)

%% Home Robot
% All angles to 0 deg
setControlMode(s, "position");
motorPos = zeros(1, numID);
sendJointPos(s, motorPos, numID);

%% Task 1 - Follow Sequence
% This is where you will write the majority of your code for part 1 of the project.
% 25 moves in sequence

%% Task 2 - Allowed Moves
% This is where you will write the majority of your code for part 2 of the project.

%% Close Serial Connection - MUST DO PRIOR TO DISCONNECTING HARDWARE OR CLEARING VARIABLES
fclose(s);

%% Test Zone
tmp = getCommand(2);
[a, b] = com2Coord(tmp);

function [th1, th2, th3, th4, th5] = trajPoints2motAng(x, y, z, d1, a2, a3, dE)
    th1 = zeros(1, length(x));
    th2 = zeros(1, length(x));
    th3 = zeros(1, length(x));
    th4 = zeros(1, length(x));
    th5 = zeros(1, length(x));

    for i = length(x)
        [th1(i), th2(i), th3(i), th4(i), th5(i)] = invKin(x, y, z, d1, a2, a3, dE);
    end

end

function [x_vals, y_vals, z_vals] = endPoints2trajPoints()

    z_rest = 1.5/100; % metres
    % Position control

    % Velocity Control
    [x_vals, y_vals, z_vals] = traj_points(time_limit, startCoord, endCoord, height);
end

function [pi, pf] = coord2endPoints(c_i, c_f, p)

    full_sq = 3.9/100; % metres
    half_sq = full_sq / 2; % metres

    % Chess board sq coord to cartesian lengths
    xi = (c_i(1) - 1) * full_sq + half_sq;
    yi = (c_i(2) - 1) * full_sq + half_sq;
    xf = (c_f(1) - 1) * full_sq + half_sq;
    yf = (c_f(2) - 1) * full_sq + half_sq;

    % Home: Start or End
    if (c_i(1) == 0)
        xi = 0;
    end

    if (c_i(2) == 0)
        yi = 0;
    end

    if (c_f(1) == 0)
        xf = 0;
    end

    if (c_f(2) == 0)
        yf = 0;
    end

    % Capture Pool
    if (c_i(1) < 0)
        xi = 9;
        yi = p;
    end

    if (c_f(1) < 0)
        xf = 9;
        yf = p;
    end

    pi = [xi, yi];
    pf = [xf, yf];

end

function [from, to] = com2Coord(com)
    c1 = double(com(1)) - double('a')
    c2 = str2double(com(2))
    c3 = double(com(3)) - double('a')
    c4 = str2double(com(4))
    from = [c1, c2]
    to = [c3, c4]
end

function nxtln = getCommand(i)
    seq = [['e2', 'e4']; ...
            ['g1', 'f3']; ...
            ['f1', 'b5']; ...
            ['f1', 'b5']; ...
            ['e1', 'g1']; ...
            ['h1', 'f1']; ...
            ['e4', 'd5']; ...
            ['d5', 'XX']; ...
            ['c3', 'd5']; ...
            ['d5', 'XX']; ...
            ['b5', 'c6']; ...
            ['c2', 'c3']; ...
            ['f3', 'g5']; ...
            ['d2', 'd4']; ...
            ['d1', 'f3']; ...
            ['f3', 'd5']; ...
            ['c1', 'XX']; ...
            ['a1', 'c1']; ...
            ['d5', 'XX']; ...
            ['g1', 'h1']; ...
            ['f2', 'f4']; ...
            ['g5', 'h3']; ...
            ['g2', 'g3']; ...
            ['c1', 'd1']; ...
            ['f1', 'e1']];
    nxtln = seq(i, :);
end

%% Assignment 3

function [th1, th2, th3, th4, th5] = invKin(x, y, z, d1, a2, a3, dE)
    th1 = atan2d(y, x);
    H = sqrt(x^2 + y^2 + (z + dE - d1)^2);

    th2 = atan2d((z + dE - d1), sqrt(x^2 + y^2)) + acosd((a2^2 + H^2 - a3^2) / (2 * a2 * H));
    th3 = acosd((a2^2 + a3^3 - H^2) / (2 * a2 * a3)) - 180;

    th4 = -th2 - th3;
    th5 = th1;
end

function [x_vals, y_vals, z_vals] = traj_points(time_limit, startCoord, endCoord, height)

    [x_coeffs, y_coeffs, z_coeffs] = traj_Coeffs(time_limit, startCoord, endCoord, height);
    x_polys = poly2funcs(x_coeffs);
    y_polys = poly2funcs(y_coeffs);
    z_polys = poly2funcs(z_coeffs);

    % Generating plotting vectors
    time = linspace(0, time_limit, resolution);

    t_seg1 = fix(resolution / 3);
    fx1 = zeros(1, t_seg1);
    fy1 = zeros(1, t_seg1);
    fz1 = zeros(1, t_seg1);

    fx2 = zeros(1, t_seg1);
    fy2 = zeros(1, t_seg1);
    fz2 = zeros(1, t_seg1);

    fx3 = zeros(1, resolution - (2 * t_seg1));
    fy3 = zeros(1, resolution - (2 * t_seg1));
    fz3 = zeros(1, resolution - (2 * t_seg1));

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

    for i = 1:resolution - (2 * t_seg1)
        fx3(n) = subs(x_polys(3), t, time(i));
        fy3(n) = subs(y_polys(3), t, time(i));
        fz3(n) = subs(z_polys(3), t, time(i));
        n = n + 1;
    end

    x_vals = [fx1, fx2, fx3];
    y_vals = [fy1, fy2, fy3];
    z_vals = [fz1, fz2, fz3];

end

function polyMat = poly2funcs(a_coeffs)
    syms t
    tmp = flip(a_coeffs);
    % coefficients for poly2sym are in order [a0,a1,a2,a3]
    % poly2sym turns vector of coefficients into a polynomcal
    for i = 1:size(tmp, 2)
        polyMat(i, 1) = poly2sym(transpose(tmp(:, i)), t);
    end

end

function [x_coeffs, y_coeffs, z_coeffs] = traj_Coeffs(time_limit, startCoord, endCoord, height)
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
    zeroVel = zeros(1, 4);
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
    t1 = t_lim / 3;

    AB = traj_seg(t0, t1, p1, p2, v1, v2);
    BC = traj_seg(t0, t1, p2, p3, v2, v3);
    CD = traj_seg(t0, t1, p3, p4, v3, v4);

    abc_coeffs = [AB, BC, CD];
end

function sols = traj_seg(t_i, t_f, p_i, p_f, v_i, v_f)
    syms t
    syms m0 m1 m2 m3
    traj = m0 + m1 * t + m2 * t^2 + m3 * t^3;
    velo = m1 + 2 * m2 * t + 3 * m3 * t^2;

    f1 = p_i == subs(traj, t, t_i);
    f2 = p_f == subs(traj, t, t_f);
    f3 = v_i == subs(velo, t, t_i);
    f4 = v_f == subs(velo, t, t_f);

    eqns = [f1, f2, f3, f4];
    coefvars = [m0, m1, m2, m3];
    [A, b] = equationsToMatrix(eqns, coefvars);
    sols = linsolve(A, b);

end
