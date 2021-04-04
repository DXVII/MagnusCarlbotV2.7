%% Robotic Systems Project 1 - Code:
% Group 27: David Pham, Pamela Kong, Thomas Pimenta


%% Creating Rotations, Translations, Augmented Matrix
syms x y z angle

% Translation
t = @(x,y,z) [x;y;z];                       

% Rotation
Rx =  @(angle)[1,0,0; 0, cos(angle),sin(angle); 0,-sin(angle),cos(angle)];
Rz = @(angle) [cos(angle),sin(angle),0; -sin(angle),cos(angle),0; 0,0,1];

% Augment
RxA = @(angle,x,y,z) [Rx(angle), t(x,y,z); 0, 0, 0, 1];
RzA = @(angle,x,y,z) [Rz(angle), t(x,y,z); 0, 0, 0, 1];

%% Part 1: Denavit Hartenberg Method
syms a alpha d theta

% Transform Chain function:  T (i-1) --> i   =  Dx(a-1)*Rx(alpha-1)*Dz(d)*Rz(theta) 
transNext = @(a, alpha, d, theta) RxA(0,a,0,0)*RxA(alpha,0,0,0)*RzA(0,0,0,d)*RzA(theta,0,0,0);


% DH Table to Transforms
% [a(i-1) alpha(i-1) d(i) theta(i)]
syms d1 th1 d2 th2 a2 d3 th3 a3 d4 th4 d5 th5 d6
DHtable =   [0 0 d1 th1; ...            % frame 0-1: XY shoulder swivel
            0 pi/2 d2 th2; ...          % frame 1-2: XZ bicep elevation
            a2 0 d3 th3; ...            % frame 2-3: XZ elbow descent
            a3 0 d4 th4; ...            % frame 3-4: XZ vertical wrist (constraint: point to ground)
            0 pi/2 d5 th5; ...          % frame 4-5: XY wrist torsional twist (constraint: point foward)
            0 0 d6 0];                  % frame 5-6: Wrist to Gripper distance 


final = eye(4);             % being DH chain

f_final = length(DHtable);  % code tings: simplify end of array 

% frame 1 to 4 chosen due to relational constrains of frames 4 and onwards (discussed in report)
frame_start = 1;            % code tings: start of transform chain
frame_stop = 4;             % code tings: end of transform chain


for row = frame_start:1:f_final    
    a = DHtable(row,1);
    alpha = DHtable(row,2);
    d = DHtable(row,3);
    theta = DHtable(row,4);
    
    fprintf("Matrix: 0 to %d\n",row)
    final = final*transNext(a, alpha, d, theta);
    finalTrans = simplify(final((1:3),4))
%   func_tab(row,1) = finalTrans(1);
%   func_tab(row,2) = finalTrans(2);
%   func_tab(row,3) = finalTrans(3);
end

