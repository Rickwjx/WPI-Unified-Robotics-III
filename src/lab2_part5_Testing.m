clear;
clc;
%% Setup robot
travelTime = 2; % Defines the travel time
robot = Robot(); % Creates robot object
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

angle1 = 34;
angle2 = 29;
angle3 = -51;
angle4 = -29;

angle1R = angle1 * (pi / 180);
angle2R = angle2 * (pi / 180);
angle3R = angle3 * (pi / 180);
angle4R = angle4 * (pi / 180);

robot.interpolate_jp(travelTime, [angle1 angle2 angle3 angle4]);
pause(travelTime);

%Intializing Model
model = Model();
%model.plot_arm([pi/4,pi/4,-pi/2,pi/2]);


model.plot_arm([angle1R, angle2R, angle3R, angle4R]);