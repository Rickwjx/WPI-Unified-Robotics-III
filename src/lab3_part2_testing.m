clear;
clc;
%% Setup robot
travelTime = 2; % Defines the travel time
robot = Robot(); % Creates robot object
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

theta1 = 45;
theta2 = 20;
theta3 = -10;
theta4 = 20;

radians = [theta1 theta2 theta3 theta4] * pi / 180;

% robot.interpolate_jp(travelTime, [theta1 theta2 theta3 theta4]);
% pause(travelTime);

fkAnswer = robot.fk3001(radians');
alpha = (theta2 + theta3 + theta4) * -1 * (pi / 180);
angles = robot.ik3001([fkAnswer(1,4) fkAnswer(2,4) fkAnswer(3,4) alpha]);
disp(angles*(180/pi));
ikAnswer = robot.fk3001(angles');

disp(fkAnswer);
disp(ikAnswer);


% L0 = 36.076;
% L1 = 96.326 - L0;
% L2 = sqrt(128^2 + 24^2);
% L3 = 124;
% L4 = 133.4;
% 
% xPos = -36.009;
% yPos = 279.0866;
% zPos = 224.3260;
% 
% theta1 = 0.1283232264;
% alpha = (24 + 18 + -12) * (pi / 180);
% 
% X4 = xPos - sin(theta1) * cos(alpha) * L4;
% Y4 = yPos - cos(theta1) * cos(alpha) * L4;
% Z4 = zPos - L4 * sin(alpha);
% 
% F = -1*(L2^2 + L3^2 - ((Z4-L1-L0)^2 + X4^2 + Y4^2))/(2*L2*L3)
