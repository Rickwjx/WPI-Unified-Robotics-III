clear
clc
%% Setup robot
robot = Robot(); % Creates robot object
robot.writeMotorState(true); % Write position mode

%% Program
robot.jacob3001([0 0 0 0])