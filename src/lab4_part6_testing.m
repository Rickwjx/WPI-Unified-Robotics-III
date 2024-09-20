clear;
clc;

%% Setup robot
trajPlanner = Traj_Planner(); % Creates traj_Planner object which also creates robot object
% robot = Robot(); % Creates Robot Object
travelTime = 2;

% trajPlanner.interpolate_jp(travelTime, [0 0 0 0]);
% pause(travelTime);

point1Angles = [0, 0, 0, 0];
point2Angles = rad2deg([0, -atan(24/128), atan(24/128)-pi/2, 0]); % To reach singularity

%Calculating Trajectory Coefficients for each joint moving from Position 1 to Position 2
trajCoef1 = trajPlanner.quintic_traj(0, 4, 0, 0,  point1Angles(1), point2Angles(1), 0, 0);
trajCoef2 = trajPlanner.quintic_traj(0, 4, 0, 0,  point1Angles(2), point2Angles(2), 0, 0);
trajCoef3 = trajPlanner.quintic_traj(0, 4, 0, 0,  point1Angles(3), point2Angles(3), 0, 0);
trajCoef4 = trajPlanner.quintic_traj(0, 4, 0, 0,  point1Angles(4), point2Angles(4), 0, 0);

%Putting all the Trajectory Coefficients into a 6x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
tjMatrix1 = trajPlanner.run_trajectory_eStop(trajCoef, 4, true);


% Plotting 3D Path of Robot
figure(2);
plot3(tjMatrix1(:,2), tjMatrix1(:,3), tjMatrix1(:,4), 'LineWidth', 2, 'Color', 'r');
ylim([-100, 100]);
grid on;
title('Path in 3D Space to Singularity');
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
zlabel('Z Position (mm)');

% Plotting Determinant vs. Time
figure(3);
plot(tjMatrix1(:, 1), tjMatrix1(:, 5), 'LineWidth', 2, 'Color', 'b');
grid on;
title('Determinant vs. Time');
xlabel('Time (s)');
ylabel('Determinant Value');