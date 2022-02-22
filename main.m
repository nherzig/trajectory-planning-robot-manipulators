%----------------------------------------------------------
%                        MAIN
%----------------------------------------------------------

clear, clc, close all;
addpath(genpath('library'));

%% Common parameters
% Load robot
robotname="abbIrb120"  % Example of some robot name "abbIrb120","frankaEmikaPanda","kinovaGen3","rethinkSawyer","universalUR5","kukaIiwa7"
robot = loadrobot(robotname,"DataFormat","column","Gravity",[0 0 -9.81]);

%Positions definitions
jointAnglesHome=robot.homeConfiguration';
eeName = char(robot.BodyNames(end));
basename=char(robot.BodyNames(1));
toolPositionHome=tform2trvec(getTransform(robot,robot.homeConfiguration,eeName,basename));
numJoints = numel(robot.homeConfiguration);
ikInitGuess = robot.homeConfiguration;

% Waypoints Positions (X Y Z)
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

% Boundary conditions (for polynomial trajectories)
% Velocity (cubic and quintic)
waypointVels = 0.1 *[ 0  0  0;
                     0  0  0;
                      0 0  0;
                      0  0  0;
                      0  0  0]';

% Acceleration (quintic only)
waypointAccels = zeros(size(waypointVels));

% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/4;

% Line width for plots
lw=2;
plotMode = 3; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames, 3 = Gif generation
%Camera angle
Az=158.1869;
El=25.3946;

%% Simulate Task-space trajectory
manipTrajCartesian

%% Simulate Joint-space trajectory
manipTrajJoint

%% Gif generation
delaytime=0.1;
if plotMode == 3
    filename = 'testAnimated.gif'; % Specify the output file name
    for idx = 1:length(im)
        [A,map] = rgb2ind(im{idx},256);
        if idx == 1
            imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',delaytime);
        else
            imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',delaytime);
        end
    end
end

%% Compare plots
compareTaskVsJointTraj