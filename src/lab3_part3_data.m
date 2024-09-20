clear;
clc;
%% Setup robot
travelTime = 5; % Defines the travel time
% robot = Robot(); % Creates robot object
model = Model(); % Creates model object which also creates robot object
robot = Robot();
%robot.writeTime(travelTime); % Write travel time
% robot.writeMotorState(true); % Write position mode

dataArray = [];
timeArray = [];
i = 0;

triPos = [200 200 115 -pi/4; 
          175 0 390 pi/4;
          185 -107 131 0];

robot.interpolate_jp(travelTime, robot.ik3001(triPos(1, :)) * 180/pi);
pause(travelTime);

% robot.interpolate_jp(travelTime, robot.ik3001(triPos(1, :)) * 180/pi);
% tic
% while toc < travelTime
%     hold off;
%     i = i + 1;
% 
%     angles = robot.setpoint_js();
% 
%     dataArray(i, :) = angles;
%     timeArray(i, :) = toc;
% 
%     angles = angles * (pi / 180);
%     model.plot_arm(angles);
%     drawnow;
% 
%     transformationMatrix = robot.fk3001((robot.setpoint_js()*pi/180)');
%     positions(i, 1) = transformationMatrix(1, 4);
%     positions(i, 2) = transformationMatrix(2, 4);
%     positions(i, 3) = transformationMatrix(3, 4);
% end


robot.interpolate_jp(travelTime, robot.ik3001(triPos(2, :)) * 180/pi);
tic
while toc < travelTime
    hold off;
    i = i + 1;

    angles = robot.setpoint_js();

    dataArray(i, :) = angles;
    timeArray(i, :) = toc;

    angles = angles * (pi / 180);
    model.plot_arm(angles);
    drawnow;

    transformationMatrix = robot.fk3001((robot.setpoint_js()*pi/180)');
    positions(i, 1) = transformationMatrix(1, 4);
    positions(i, 2) = transformationMatrix(2, 4);
    positions(i, 3) = transformationMatrix(3, 4);
end

point2TransMat = transformationMatrix;


robot.interpolate_jp(travelTime, robot.ik3001(triPos(3, :)) * 180/pi);
nextTravelTime = toc + travelTime;
while toc < nextTravelTime
    hold off;
    i = i + 1;

    angles = robot.setpoint_js();

    dataArray(i, :) = angles;
    timeArray(i, :) = toc;

    angles = angles * (pi / 180);
    model.plot_arm(angles);
    drawnow;

    transformationMatrix = robot.fk3001((robot.setpoint_js()*pi/180)');
    positions(i, 1) = transformationMatrix(1, 4);
    positions(i, 2) = transformationMatrix(2, 4);
    positions(i, 3) = transformationMatrix(3, 4);
end

point3TransMat = transformationMatrix;


robot.interpolate_jp(travelTime, robot.ik3001(triPos(1, :)) * 180/pi);
nextTravelTime = toc + travelTime;
while toc < nextTravelTime
    hold off;
    i = i + 1;

    angles = robot.setpoint_js();

    dataArray(i, :) = angles;
    timeArray(i, :) = toc;

    angles = angles * (pi / 180);
    model.plot_arm(angles);
    drawnow;

    transformationMatrix = robot.fk3001((robot.setpoint_js()*pi/180)');
    positions(i, 1) = transformationMatrix(1, 4);
    positions(i, 2) = transformationMatrix(2, 4);
    positions(i, 3) = transformationMatrix(3, 4);
end

point1TransMat = transformationMatrix;

hold off;

%Joint angles vs. time
figure(1);
plot(timeArray(:,1), dataArray(:,1), 'LineWidth', 2, 'Color', 'r');
title('Joint Positions vs. Time');
xlabel('Time (s)');
ylabel('Joint Position (Degrees)');

hold on;

plot(timeArray(:,1), dataArray(:,2), 'LineWidth', 2, 'Color', 'g');

plot(timeArray(:,1), dataArray(:,3), 'LineWidth', 2, 'Color', 'b');

plot(timeArray(:,1), dataArray(:,4), 'LineWidth', 2, 'Color', 'm');

legend('Joint 1','Joint 2', 'Joint 3', 'Joint 4');
grid on;

hold off;

%End effector positions vs time
figure(2);
plot(timeArray(:,1), positions(:,1), 'LineWidth', 2, 'Color', 'r');
title('End-Effector Positions vs. Time');
xlabel('Time (s)');
ylabel('Coordinate value (mm)');

hold on;

plot(timeArray(:,1), positions(:,2), 'LineWidth', 2, 'Color', 'g');

plot(timeArray(:,1), positions(:,3), 'LineWidth', 2, 'Color', 'b');

legend('X Values', 'Y Values', 'Z Values');
grid on;

hold off;

% Measured Path in 3D Space
figure(3);
plot3(positions(:,1), positions(:,2), positions(:,3));
grid on;
title('Part 3 3D Path')
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
zlabel('Z Position (mm)');

