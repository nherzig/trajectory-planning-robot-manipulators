## Trajectory Planning of Robot Manipulators with MATLAB

## Description

This repository provides a tool to compare task-space trajectory with joint space trajectories. 

## Files
To get started, run the `main.m` script. This will configure the MATLAB search path so all the examples run correctly.

### `library` Folder
Contains MATLAB examples for trajectory planning.

* `manipTrajCartesian.m` - Task space (translation only) trajectories
* `manipTrajJoint.m` - Joint space trajectories. Contains an `includeOrientation` variable to toggle waypoint orientations on or off. 
* `manipTrajLinearRotation.m` - Task space (translation only) trajectories with linearly interpolated orientation
* `manipTrajTransform.m` - Linearly interpolated transform trajectories (translation and orientation) 
* `manipTrajTransformTimeScaling.m` - Transform trajectories (translation and orientation) interpolated using nonlinear time scaling
* `compareTaskVsJointTraj.m` - Comparison script that illustrates the difference between task space and joint space trajectories

## Acknowledgement

The proposed example has been forked and adapted from https://github.com/mathworks-robotics/trajectory-planning-robot-manipulators by Sebastian Castro (sea-bass)
