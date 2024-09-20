clear;
clc;
%% Setup robot
travelTime = 2; % Defines the travel time
robot = Robot(); % Creates robot object
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
dataArray = [];
timeArray = [];
positions = [];
triangle = [];
i = 0;

robot.interpolate_jp(travelTime, [0 0 0 0]);
tic
while toc < travelTime
    i = i + 1;
    dataArray(i, :) = robot.setpoint_js();
    timeArray(i, :) = toc;

    transformationMatrix = robot.fk3001((robot.setpoint_js()*pi/180)');
    positions(i, 1) = transformationMatrix(1, 4);
    positions(i, 2) = transformationMatrix(2, 4);
    positions(i, 3) = transformationMatrix(3, 4);
end

triangle(1, 1) = transformationMatrix(1, 4);
triangle(1, 2) = transformationMatrix(2, 4);
triangle(1, 3) = transformationMatrix(3, 4);

% i = i + 1;
% dataArray(i, :) = [0 0 0 0];
% timeArray(i, :) = 0;
% positions(i, :) = [0 0 0];


robot.interpolate_jp(travelTime, [0 -15 -30 -15]);
nextTravelTime = toc + travelTime;
while toc < nextTravelTime
    i = i + 1;
    dataArray(i, :) = robot.setpoint_js();
    timeArray(i, :) = toc;

    transformationMatrix = robot.fk3001((robot.setpoint_js()*pi/180)');
    positions(i, 1) = transformationMatrix(1, 4);
    positions(i, 2) = transformationMatrix(2, 4);
    positions(i, 3) = transformationMatrix(3, 4);
end

triangle(2, 1) = transformationMatrix(1, 4);
triangle(2, 2) = transformationMatrix(2, 4);
triangle(2, 3) = transformationMatrix(3, 4);

% i = i + 1;
% dataArray(i, :) = [0 0 0 0];
% timeArray(i, :) = 0;
% positions(i, :) = [0 0 0];


robot.interpolate_jp(travelTime, [0 0 40 10]);
nextTravelTime = toc + travelTime;
while toc < nextTravelTime
    i = i + 1;
    dataArray(i, :) = robot.setpoint_js();
    timeArray(i, :) = toc;

    transformationMatrix = robot.fk3001((robot.setpoint_js()*pi/180)');
    positions(i, 1) = transformationMatrix(1, 4);
    positions(i, 2) = transformationMatrix(2, 4);
    positions(i, 3) = transformationMatrix(3, 4);
end

triangle(3, 1) = transformationMatrix(1, 4);
triangle(3, 2) = transformationMatrix(2, 4);
triangle(3, 3) = transformationMatrix(3, 4);

% i = i + 1;
% dataArray(i, :) = [0 0 0 0];
% timeArray(i, :) = 0;
% positions(i, :) = [0 0 0];


robot.interpolate_jp(travelTime, [0 0 0 0]);
nextTravelTime = toc + travelTime;
while toc < nextTravelTime
    i = i + 1;
    dataArray(i, :) = robot.setpoint_js();
    timeArray(i, :) = toc;

    transformationMatrix = robot.fk3001((robot.setpoint_js()*pi/180)');
    positions(i, 1) = transformationMatrix(1, 4);
    positions(i, 2) = transformationMatrix(2, 4);
    positions(i, 3) = transformationMatrix(3, 4);
end

triangle(4, 1) = transformationMatrix(1, 4);
triangle(4, 2) = transformationMatrix(2, 4);
triangle(4, 3) = transformationMatrix(3, 4);

figure(1);
plot(timeArray(:,1), dataArray(:,2), 'LineWidth', 2, 'Color', 'r');
hold on;
plot(timeArray(:,1), dataArray(:,3), 'LineWidth', 2, 'Color', 'g');
plot(timeArray(:,1), dataArray(:,4), 'LineWidth', 2, 'Color', 'b');
title('Joint Angles 2, 3, and 4 vs. Time');
xlabel('Time (s)');
ylabel('Joint Angles (Degrees)');
legend({'Joint 2','Joint 3', 'Joint 4'},'Location','southwest');

figure(2);
plot(timeArray(:,1), positions(:,1), 'LineWidth', 2, 'Color', 'r');
hold on;
plot(timeArray(:,1), positions(:,3), 'LineWidth', 2, 'Color', 'g');
title('End Effector X and Z Positions vs. Time');
xlabel('Time (s)');
ylabel('End Effector Positions (mm)');
legend({'X Position','Z Position'},'Location','southwest');

figure(3);
plot(positions(:,1), positions(:,3), 'LineWidth', 2, 'Color', 'b');
hold on;
plot(triangle(1,1), triangle(1,3), 'Marker', '*', 'Color', 'r');
plot(triangle(2,1), triangle(2,3), 'Marker', '*', 'Color', 'g');
plot(triangle(3,1), triangle(3,3), 'Marker', '*', 'Color', 'm');
title('End Effector Z Position vs. X Position');
xlabel('X Position (mm)');
ylabel('Z Position (mm)');

figure(4);
plot(positions(:,1), positions(:,2), 'LineWidth', 2, 'Color', 'b');
title('End Effector Y Position vs. X Position');
xlabel('X Position (mm)');
ylabel('Y Position (mm)');