% Create sample waypoint data for trajectory generation
% NOTE: Modify this script with your own rigid body tree and 
%       trajectory reference points
%       (We recommend saving a copy of this file)
%
% Copyright 2019 The MathWorks, Inc.

%% Common parameters
% Rigid Body Tree information
robot = loadrobot("abbIrb120","DataFormat","column","Gravity",[0 0 -9.81]);

%Positions definitions
jointAnglesHome=robot.homeConfiguration';
eeName = char(robot.BodyNames(end));
basename=char(robot.BodyNames(1));
toolPositionHome=tform2trvec(getTransform(robot,robot.homeConfiguration,eeName,basename));
numJoints = numel(robot.homeConfiguration);
ikInitGuess = robot.homeConfiguration;

% Maximum number of waypoints (for Simulink)
maxWaypoints = 20;

% Positions (X Y Z)
waypoints = [0.374-0.1 0 0.63]' + ... 
            [0 -0.2 -0.1 ; 0 -0.2 -0.4 ; 0 0.2 -0.4 ; 0 0.2 -0.1 ; 0 -0.2 -0.1]';
         
% Euler Angles (Z Y X) relative to the home orientation       
orientations = [0     0    0;
                0  0    0; 
                0    0  0;
               0  0    0;
                0     0    0]';   
            
% Array of waypoint times
waypointTimes = 0:4:16;

% Trajectory sample time
ts = 0.2;
trajTimes = 0:ts:waypointTimes(end);

%% Additional parameters

% Boundary conditions (for polynomial trajectories)
% Velocity (cubic and quintic)
waypointVels = 0.1 *[ 0  1  0;
                     -1  0  0;
                      0 -1  0;
                      1  0  0;
                      0  1  0]';

% Acceleration (quintic only)
waypointAccels = zeros(size(waypointVels));

% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/4;
