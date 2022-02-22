% MANIPULATOR TRAJECTORY GENERATION
% Generates Cartesian only (no rotation) trajectories
%
% Copyright 2019 The MathWorks, Inc.

%% Setup

% Define IK
ik = inverseKinematics('RigidBodyTree',robot);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = robot.homeConfiguration;

% Set up plot

show(robot,jointAnglesHome','Frames','off','PreservePlot',false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1||plotMode == 3
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

view(Az,El)

%% Generate trajectory
% Cartesian Motion only
trajType = 'trap';
switch trajType
    case 'trap'
        [q,qd,qdd] = trapveltraj(waypoints,numel(trajTimes), ...
            'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
            'EndTime',repmat(diff(waypointTimes),[3 1]));
                            
    case 'cubic'
        [q,qd,qdd] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels);
        
    case 'quintic'
        [q,qd,qdd] = quinticpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels, ...
            'AccelerationBoundaryCondition',waypointAccels);
        
    case 'bspline'
        ctrlpoints = waypoints; % Can adapt this as needed
        [q,qd,qdd] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
        
    otherwise
        error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
end

% Show the full trajectory with the rigid body tree
if plotMode == 3
    fig=gcf
elseif plotMode == 2
    plotTransforms(q',repmat([1 0 0 0],[size(q,2) 1]),'FrameSize',0.05);
end

% To visualize the trajectory, run the following line
% plotTrajectory(trajTimes,q,qd,qdd,'Names',["X","Y","Z"],'WaypointTimes',waypointTimes)

%% Trajectory following loop
for idx = 1:numel(trajTimes) 
    % Solve IK
    tgtPose = trvec2tform(q(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;
    if plotMode == 1||plotMode == 3
        set(hTraj,'xdata',q(1,1:idx), ...
                  'ydata',q(2,1:idx), ...
                  'zdata',q(3,1:idx));
    end
    
    % Show the robot
    show(robot,config,'Frames','off','PreservePlot',false);
    title(['Task space Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow    
    if plotMode == 3
        frame = getframe(fig);
        im{idx} = frame2im(frame);
    end
end