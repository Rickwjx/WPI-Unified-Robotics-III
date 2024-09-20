clear;
clc;
%% Setup robot
travelTime = 2; % Defines the travel time
robot = Robot(); % Creates robot object
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

%% Program 

%syms theta1 theta2 theta3 theta4
%dhMatrix = [30, 45, 50, 55];

jointAngles = [pi, pi/4, pi/36, pi];
jointAngles2 = [0 0 0 0];

robot.fk3001(jointAngles2);
