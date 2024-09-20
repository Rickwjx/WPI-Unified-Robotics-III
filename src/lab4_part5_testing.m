clear;
clc;

%% Setup robot
model = Model(); % Creates Model Object
trajPlanner = Traj_Planner(); % Creates traj_Planner object which also creates robot object
robot = Robot(); % Creates Robot Object
travelTime = 4;

% Task Space Trajectory Movement
point1 = [200 200 115 -pi/4];
point2 = [175 0 390 pi/4];
point3 = [185 -107 131 0];

% Calculating Trajectory Coefficients for each joint moving from Position 1 to Position 2
trajCoef1 = trajPlanner.quintic_traj(0, 4, 0, 0,  point1(1), point2(1), 0, 0);
trajCoef2 = trajPlanner.quintic_traj(0, 4, 0, 0,  point1(2), point2(2), 0, 0);
trajCoef3 = trajPlanner.quintic_traj(0, 4, 0, 0,  point1(3), point2(3), 0, 0);
trajCoef4 = trajPlanner.quintic_traj(0, 4, 0, 0,  point1(4), point2(4), 0, 0);

% Putting all the Trajectory Coefficients into a 4x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
% Running the Trajectory 
tjMatrix1 = trajPlanner.run_trajectory_plot(trajCoef, 4, false);

% Calculating Trajectory Coefficients for each joint moving from Position 2 to Position 3
trajCoef1 = trajPlanner.quintic_traj(0, 4, 0, 0,  point2(1), point3(1), 0, 0);
trajCoef2 = trajPlanner.quintic_traj(0, 4, 0, 0,  point2(2), point3(2), 0, 0);
trajCoef3 = trajPlanner.quintic_traj(0, 4, 0, 0,  point2(3), point3(3), 0, 0);
trajCoef4 = trajPlanner.quintic_traj(0, 4, 0, 0,  point2(4), point3(4), 0, 0);

% Putting all the Trajectory Coefficients into a 4x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
% Running the Trajectory 
tjMatrix2 = trajPlanner.run_trajectory_plot(trajCoef, 4, false);

% Calculating Trajectory Coefficients for each joint moving from Position 3 to Position 1
trajCoef1 = trajPlanner.quintic_traj(0, 4, 0, 0,  point3(1), point1(1), 0, 0);
trajCoef2 = trajPlanner.quintic_traj(0, 4, 0, 0,  point3(2), point1(2), 0, 0);
trajCoef3 = trajPlanner.quintic_traj(0, 4, 0, 0,  point3(3), point1(3), 0, 0);
trajCoef4 = trajPlanner.quintic_traj(0, 4, 0, 0,  point3(4), point1(4), 0, 0);

% Putting all the Trajectory Coefficients into a 4x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
% Running the Trajectory 
tjMatrix3 = trajPlanner.run_trajectory_plot(trajCoef, 4, false);

% Calculating Trajectory Coefficients for each joint moving from Position 1 to Position 1
trajCoef1 = trajPlanner.quintic_traj(0, 1, 0, 0,  point1(1), point1(1), 0, 0);
trajCoef2 = trajPlanner.quintic_traj(0, 1, 0, 0,  point1(2), point1(2), 0, 0);
trajCoef3 = trajPlanner.quintic_traj(0, 1, 0, 0,  point1(3), point1(3), 0, 0);
trajCoef4 = trajPlanner.quintic_traj(0, 1, 0, 0,  point1(4), point1(4), 0, 0);

% Putting all the Trajectory Coefficients into a 4x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
% Running the Trajectory
tjMatrix4 = trajPlanner.run_trajectory_plot(trajCoef, 1, false);




% Plotting Velocities
% Combining Time to be Continuous
tjMatrix2(:,1) = tjMatrix2(:,1) + tjMatrix1(height(tjMatrix1), 1);
tjMatrix3(:,1) = tjMatrix3(:,1) + tjMatrix2(height(tjMatrix2), 1);
tjMatrix4(:,1) = tjMatrix4(:,1) + tjMatrix3(height(tjMatrix3), 1);

% Combining tjMatrices
tjMatrixTotal = vertcat(tjMatrix1, tjMatrix2, tjMatrix3, tjMatrix4);

% Plotting linear velocity
figure(2);
plot(tjMatrixTotal(:, 1), tjMatrixTotal(:, 2), 'LineWidth', 2, 'Color', 'r');
grid on;
hold on;
plot(tjMatrixTotal(:, 1), tjMatrixTotal(:, 3), 'LineWidth', 2, 'Color', 'g');
plot(tjMatrixTotal(:, 1), tjMatrixTotal(:, 4), 'LineWidth', 2, 'Color', 'b');
title('Part 5 Linear Velocity vs. Time');
xlabel('Time (s)');
ylabel('End Effector Linear Velocity (mm/s)');
legend('X Linear Velocity', 'Y Linear Velocity', 'Z Linear Velocity');

% Plotting angular velocity
figure(3);
plot(tjMatrixTotal(:, 1), tjMatrixTotal(:, 5), 'LineWidth', 2, 'Color', 'r');
grid on;
hold on;
plot(tjMatrixTotal(:, 1), tjMatrixTotal(:, 6), 'LineWidth', 2, 'Color', 'g');
plot(tjMatrixTotal(:, 1), tjMatrixTotal(:, 7), 'LineWidth', 2, 'Color', 'b');
title('Part 5 Angular Velocity vs. Time');
xlabel('Time (s)');
ylabel('End Effector Angular Velocity (rad/s)');
legend('X Angular Velocity', 'Y Linear Velocity', 'Z Linear Velocity');

% Plotting magnitude
figure(4);
plot(tjMatrixTotal(:, 1), tjMatrixTotal(:, 8), 'LineWidth', 2, 'Color', 'r');
grid on;
hold on;
title('Part 5 Velocity Magnitude vs. Time');
xlabel('Time (s)');
ylabel('End Effector Magnitude (mm/s)');