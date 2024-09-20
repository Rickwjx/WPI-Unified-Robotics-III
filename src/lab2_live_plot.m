model = Model(); % Creates a model which also creates a robot
% robot = Robot();
travelTime = 2; % Defines the travel time
dataArray = [];
timeArray = [];
i = 0;

% while 2 > 0
%     pause(0.5);
%     angles = robot.setpoint_js();
%     angles = angles * (pi / 180);
%     model.plot_arm(angles);
%     drawnow;
% end

robot.interpolate_jp(travelTime, [0 0 0 0]);
tic
while toc < travelTime
    hold off;
    angles = robot.setpoint_js();
    angles = angles * (pi / 180);
    model.plot_arm(angles);
    drawnow;

    i = i + 1;
    dataArray(i, :) = robot.setpoint_js();
    timeArray(i, :) = toc;
end

fk1 = robot.fk3001((robot.setpoint_js()*pi/180)');

% i = i + 1;
% dataArray(i, :) = [0 0 0 0];
% timeArray(i, :) = 0;

robot.interpolate_jp(travelTime, [34 29 -51 -29]);
nextTravelTime = toc + travelTime;
while toc < nextTravelTime
    hold off;
    angles = robot.setpoint_js();
    angles = angles * (pi / 180);
    model.plot_arm(angles);
    drawnow;

    i = i + 1;
    dataArray(i, :) = robot.setpoint_js();
    timeArray(i, :) = toc;
end

fk2 = robot.fk3001((robot.setpoint_js()*pi/180)');

% i = i + 1;
% dataArray(i, :) = [0 0 0 0];
% timeArray(i, :) = 0;

robot.interpolate_jp(travelTime, [23 -50 19 -54]);
nextTravelTime = toc + travelTime;
while toc < nextTravelTime
    hold off;
    angles = robot.setpoint_js();
    angles = angles * (pi / 180);
    model.plot_arm(angles);
    drawnow;

    i = i + 1;
    dataArray(i, :) = robot.setpoint_js();
    timeArray(i, :) = toc;
end

fk3 = robot.fk3001((robot.setpoint_js()*pi/180)');

% i = i + 1;
% dataArray(i, :) = [0 0 0 0];
% timeArray(i, :) = 0;

robot.interpolate_jp(travelTime, [34 -44 -30 27]);
nextTravelTime = toc + travelTime;
while toc < nextTravelTime
    hold off;
    angles = robot.setpoint_js();
    angles = angles * (pi / 180);
    model.plot_arm(angles);
    drawnow;

    i = i + 1;
    dataArray(i, :) = robot.setpoint_js();
    timeArray(i, :) = toc;
end

fk4 = robot.fk3001((robot.setpoint_js()*pi/180)');

% i = i + 1;
% dataArray(i, :) = [0 0 0 0];
% timeArray(i, :) = 0;

robot.interpolate_jp(travelTime, [-20 -44 -48 35]);
nextTravelTime = toc + travelTime;
while toc < nextTravelTime
    hold off;
    angles = robot.setpoint_js();
    angles = angles * (pi / 180);
    model.plot_arm(angles);
    drawnow;

    i = i + 1;
    dataArray(i, :) = robot.setpoint_js();
    timeArray(i, :) = toc;
end

fk5 = robot.fk3001((robot.setpoint_js()*pi/180)');

% i = i + 1;
% dataArray(i, :) = [0 0 0 0];
% timeArray(i, :) = 0;

robot.interpolate_jp(travelTime, [-42 22 38 -42]);
nextTravelTime = toc + travelTime;
while toc < nextTravelTime
    hold off;
    angles = robot.setpoint_js();
    angles = angles * (pi / 180);
    model.plot_arm(angles);
    drawnow;

    i = i + 1;
    dataArray(i, :) = robot.setpoint_js();
    timeArray(i, :) = toc;
end

fk6 = robot.fk3001((robot.setpoint_js()*pi/180)');

% i = i + 1;
% dataArray(i, :) = [0 0 0 0];
% timeArray(i, :) = 0;

robot.interpolate_jp(travelTime, [0 0 0 0]);
nextTravelTime = toc + travelTime;
while toc < nextTravelTime
    hold off;
    angles = robot.setpoint_js();
    angles = angles * (pi / 180);
    model.plot_arm(angles);
    drawnow;

    i = i + 1;
    dataArray(i, :) = robot.setpoint_js();
    timeArray(i, :) = toc;
end

fk7 = robot.fk3001((robot.setpoint_js()*pi/180)');

hold off;

figure(1);
plot(timeArray(:,1), dataArray(:,1), 'LineWidth', 2, 'Color', 'b');
title('Joint 1 vs. Time');
xlabel('Time (s)');
ylabel('Joint Position (Degrees)');

figure(2);
plot(timeArray(:,1), dataArray(:,2), 'LineWidth', 2, 'Color', 'b');
title('Joint 2 vs. Time');
xlabel('Time (s)');
ylabel('Joint Position (Degrees)');

figure(3);
plot(timeArray(:,1), dataArray(:,3), 'LineWidth', 2, 'Color', 'b');
title('Joint 3 vs. Time');
xlabel('Time (s)');
ylabel('Joint Position (Degrees)');

figure(4);
plot(timeArray(:,1), dataArray(:,4), 'LineWidth', 2, 'Color', 'b');
title('Joint 4 vs. Time');
xlabel('Time (s)');
ylabel('Joint Position (Degrees)');