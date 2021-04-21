clear all
close all
%% Initial and Final Pose Defintions
R_0_i = eye(3,3);

R_0_f = [0 -1 0; 1 0 0; 0 0 1];
R_0_f = eul2rotm(0.65, 0.36, -1.45);

% Plot
figure(); subplot(2,1,1); hold on;
trplot([R_0_i [0;0;0];[0 0 0 1]], 'rgb', 'rviz', 'frame', 'i');
trplot([R_0_f [4;0;0];[0 0 0 1]], 'rgb', 'rviz', 'frame', 'i');
xlim([-1 6]);
ylim([-1 1]);
zlim([-1 1]);
axis equal
grid on
pause;

%% Rotation from Init to Final:
R_i_f = R_0_i' * R_0_f;             % Eq (4)

% Convert this TRAJECTORY rotation matrix to Euler parameters (vector and angle)
[theta_f, k] = tr2angvec(R_i_f);    % Eq 5 & 6

% Plot it (in the middle for clarity)

P1 = [4;0;0]/2;
P2 = P1+k';
h = plot_arrow(p1, p2);
t = text(P2(1), P2(2), P2(3), ['k (\theta from 0 to ' num2str(theta_f, '%.2f') ')' ]); % t.Iterpreter = ..?
% axis auto
pause;

%% Cubic Polynomial - My Parameter Theta interpolated
t_f = 5; % 5 seconds movement
t = 0:0.01:t_f;
theta = 0 + 0 + 3*theta_f.*t.^2/t_f^2 - 2*theta_f.*t.^3/t_f^3; % cubic polynomial with theta(0) = 0 and dtheta(0) = dtheta(t_f) = ...?
% Plot
subplot(2,1,2);
plot(t,theta);
text(0.05, 0.1, '\theta(0)=\theta_i=0');
text(t_f, theta_f,'theta(t_f) = \theta_f');
text(t_f/2+0.1,theta_f/2,'\theta(t)=1_0 + 1_1t + 1_2t^2 + a_3t^3');
title('/theta interpolation (here cubic) over trajectory time. k remains constant');
ylabel('\theta (t)');
xlabel('t (in s)');
pause;

%% For each intant, get rotation matrix:

R_i_t1 = angvec2tr(theta(round(end/4)), k); % Eq (8) & (9)
R_0_t1 = R_0_i*R_i_t1(1:3,1:3); % Eq 10
subplot(2,1,1); trplot([R_0_t1 [0;0;0];[0 0 0 1]],'rgb', 'frame', 't1');
subplot(2,1,2); hold on; plot(t_f/4, theta(round(end/4)), 'o');
pause;

R_i_t2 = angvec2tr(theta(round(end/2)), k); % Eq (8) & (9)
R_0_t2 = R_0_i*R_i_t2(1:3,1:3); % Eq 10
subplot(2,1,1); trplot([R_0_t2 [2;0;0];[0 0 0 1]],'rgb', 'frame', 't2');
subplot(2,1,2); hold on; plot(t_f/2, theta(round(end/2)), 'o');
pause;

R_i_t3 = angvec2tr(theta(round(end/4)), k); % Eq (8) & (9)
R_0_t3 = R_0_i*R_i_t3(1:3,1:3); % Eq 10
subplot(2,1,1); trplot([R_0_t3 [3;0;0];[0 0 0 1]],'rgb', 'frame', 't3');
subplot(2,1,2); hold on; plot(3&t_f/4, theta(3*round(end/4)), 'o');