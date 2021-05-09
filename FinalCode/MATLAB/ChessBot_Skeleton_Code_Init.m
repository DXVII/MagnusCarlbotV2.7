% ChessBot_Skeleton_Code.m
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



How To Use This Skeleton Code

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


Establish Serial Connection
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
% port = '/dev/cu.usbmodem14141';                   % MacOS Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS
port = 'COM10';                                     % Windows Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS

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

Sample Code for Reading Joystick Input
% Read joystick input
[xJoy, yJoy, eJoy] = readJoy(s);
[xJoy, yJoy]                                    % Display joystick input


Sample Code for Reading Motor Feedback
% Read motor position feedback
[motorFB, eFB] = readFB(s, numID);
motorFB                                         % Display motor feedback


Sample Code for Changing To Position Control Mode
% Set motor EPROM to position control mode
setControlMode(s, "position");


Sample Code for Sending Position Commands - Motors Must Be In Position Control Mode
% Send basic speed up and slow down trajectory to motors
for v = [0:0.001:1.8762 1.8762:-0.001:-1.8762 -1.8762:0.001:0]
    
    motorPos = zeros(1, numID) + v;
    
    % To control each motor individually change the above line to be in a format like this:
    % motorVel = [q0 q1 q2 q3 q4 q5];
    
    sendJointPos(s, motorPos, numID);
end



Sample Code for Changing To Velocity Control Mode
% Set motor EPROM to velocity control mode
setControlMode(s, "velocity");

Sample Code for Sending Velocity Commands - Motors Must Be In Velocity Control Mode
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


Sample Code for Setting Motor PID - Only Valid for Position Control Mode
% Set Motor internal PID controller. Gains must be integers.
tmpID = 1;
Kp = 10;
Ki = 0;
Kd = 1;
setPID(s, tmpID, Kp, Ki, Kd)



Change Motor ID (ONLY USE IF ABSOLUTELY NECESSARY - This should not be needed except for specific debugging scenarios)
% Change the ID number of a connected motor. NOTE: This should only be done with ONE MOTOR connected 
% at a time to ensure other IDs are not affected.


% ///////////////////////////////////////////////////// %
% /// This is the new ID you want the motor to have /// %

newID = 8;      % CHANGE THIS VALUE!    

% ///////////////////////////////////////////////////// %

if changeMotorID == true && numID == 1
    oldID = ID(1);  % This is the current ID of the motor you wish to change.
    changeID(s, oldID, newID);
    
    % Close serial connection. Establish Serial Connection section needs to be rerun.
    fclose(s);
    
    "ID Change Complete." 
    "Please re-run Establish Serial Connection section."
    
elseif changeMotorID == false
    fclose(s);
    "ID Change Failed. Please ensure flag is set to true!"
    "Please re-run Establish Serial Connection section."
elseif numID > 1
    fclose(s);
    "ID Change Failed. Please ensure flag is set to true and only one motor is connected!"
    "Please re-run Establish Serial Connection section."
end



Home Robot
% This is where you will write a sequence to send all of your joints to their home positions.








Part 1
% This is where you will write the majority of your code for part 1 of the project. 









Part 2
% This is where you will write the majority of your code for part 2 of the project. 








Close Serial Connection - MUST DO PRIOR TO DISCONNECTING HARDWARE OR CLEARING VARIABLES
fclose(s);


%% Sequence
[['e2','e4']; ...
['g1','f3']; ...
['f1','b5']; ...
['f1','b5']; ...
['e1','g1']; ...
['h1','f1']; ...
['e4','d5']; ...
['d5','XX']; ...
['c3','d5']; ...
['d5','XX']; ...
['b5','c6']; ...
['c2','c3']; ...
['f3','g5']; ...
['d2','d4']; ...
['d1','f3']; ...
['f3','d5']; ...
['c1','XX']; ...
['a1','c1']; ...
['d5','XX']; ...
['g1','h1']; ...
['f2','f4']; ...
['g5','h3']; ...
['g2','g3']; ...
['c1','d1']; ...
['f1','e1']]
