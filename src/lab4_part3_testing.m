clear
clc
%% Setup robot
robot = Robot(); % Creates robot object
robot.writeMotorState(true); % Write position mode

%% Program


% robot.interpolate_jp(2, [0 0 rad2deg(-pi/2) 0]);
% pause(2);

j = robot.jacob3001(deg2rad([0 -10.61965528 -79.38034472 0]));

bruh = det(j(1:3,1:3))