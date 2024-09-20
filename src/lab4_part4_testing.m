%% Setup robot
robot = Robot(); % Creates robot object
robot.writeMotorState(true); % Write position mode
travelTime = 5;

robot.interpolate_jp(2, [0 0 0 0]);
pause(2);

robot.interpolate_jp(travelTime, [45 -30 20 -28]);
tic;
while toc < travelTime
    robot.fdk3001(deg2rad(robot.setpoint_js()));
end