clear;
clc;
%% Setup robot
travelTime = 5; % Defines the travel time
% robot = Robot(); % Creates robot object
trajPlanner = Traj_Planner(); % Creates traj_Planner object which also creates robot object
robot = Robot();

%Get joint angles of each vertex of the triangle

homePosition = [0, 0 ,0 ,0];
point1Angles = robot.ik3001([200 200 115 -pi/4]) * (180 / pi);
point2Angles = robot.ik3001([175 0 390 pi/4]) * (180 / pi);
point3Angles = robot.ik3001([185 -107 131 0]) * (180 / pi);



%Calculating Trajectory Coefficients for each joint moving from Position 1 to Position 2
trajCoef1 = trajPlanner.quintic_traj(0, 4, 0, 0,  point1Angles(1), point2Angles(1), 0, 0);
trajCoef2 = trajPlanner.quintic_traj(0, 4, 0, 0,  point1Angles(2), point2Angles(2), 0, 0);
trajCoef3 = trajPlanner.quintic_traj(0, 4, 0, 0,  point1Angles(3), point2Angles(3), 0, 0);
trajCoef4 = trajPlanner.quintic_traj(0, 4, 0, 0,  point1Angles(4), point2Angles(4), 0, 0);

%Putting all the Trajectory Coefficients into a 6x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
tjMatrix2 = trajPlanner.run_trajectory(trajCoef, 4, true);




%Calculating Trajectory Coefficients for each joint moving from Position 2 to Position 3
trajCoef1 = trajPlanner.quintic_traj(0, 4, 0, 0, point2Angles(1), point3Angles(1), 0, 0);
trajCoef2 = trajPlanner.quintic_traj(0, 4, 0, 0,  point2Angles(2), point3Angles(2), 0, 0);
trajCoef3 = trajPlanner.quintic_traj(0, 4, 0, 0,  point2Angles(3), point3Angles(3), 0, 0);
trajCoef4 = trajPlanner.quintic_traj(0, 4, 0, 0,  point2Angles(4), point3Angles(4), 0, 0);

%Putting all the Trajectory Coefficients into a 6x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
tjMatrix3 = trajPlanner.run_trajectory(trajCoef, 4, true);




%Calculating Trajectory Coefficients for each joint moving from Position 3 to Position 
trajCoef1 = trajPlanner.quintic_traj(0, 4, 0, 0,  point3Angles(1), point1Angles(1), 0, 0);
trajCoef2 = trajPlanner.quintic_traj(0, 4, 0, 0,  point3Angles(2), point1Angles(2), 0, 0);
trajCoef3 = trajPlanner.quintic_traj(0, 4, 0, 0,  point3Angles(3), point1Angles(3), 0, 0);
trajCoef4 = trajPlanner.quintic_traj(0, 4, 0, 0,  point3Angles(4), point1Angles(4), 0, 0);

%Putting all the Trajectory Coefficients into a 6x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
tjMatrix4 = trajPlanner.run_trajectory(trajCoef, 4, true);







% Plotting Total Joint Movement
% Combining Time to be Continuous
% tjMatrix2(:,5) = tjMatrix2(:,5) + tjMatrix1(height(tjMatrix1), 5);
tjMatrix3(:,5) = tjMatrix3(:,5) + tjMatrix2(height(tjMatrix2), 5);
tjMatrix4(:,5) = tjMatrix4(:,5) + tjMatrix3(height(tjMatrix3), 5);

tjMatrixTotal = vertcat(tjMatrix2, tjMatrix3, tjMatrix4);

figure(1);
plot(tjMatrixTotal(:, 5), tjMatrixTotal(:,1), 'LineWidth', 2, 'Color', 'r');
grid on;
hold on;
plot(tjMatrixTotal(:, 5), tjMatrixTotal(:,2), 'LineWidth', 2, 'Color', 'g');
plot(tjMatrixTotal(:, 5), tjMatrixTotal(:,3), 'LineWidth', 2, 'Color', 'b');
plot(tjMatrixTotal(:, 5), tjMatrixTotal(:,4), 'LineWidth', 2, 'Color', 'm');
title('Part 7 Joint Position vs. Time');
xlabel('Time (s)');
ylabel('Joint Position (Degrees)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4');



% Plotting Total Position, Velocity, and Acceleration
% Converting Joint Angles into Task Space for Plotting
for i = 1:height(tjMatrixTotal)
    fkPosition = trajPlanner.fk3001(([tjMatrixTotal(i, 1), tjMatrixTotal(i, 2), tjMatrixTotal(i, 3), tjMatrixTotal(i, 4)] * pi / 180)');
    fkPositionXYZ(i, 1) = fkPosition(1, 4);
    fkPositionXYZ(i, 2) = fkPosition(2, 4);
    fkPositionXYZ(i, 3) = fkPosition(3, 4);
end

% Calculating Velocity
for i = 1:height(fkPositionXYZ)-1
    fkVelocityXYZ(i, 1) = (fkPositionXYZ(i+1, 1) - fkPositionXYZ(i, 1)) / (tjMatrixTotal(i + 1, 5) - tjMatrixTotal(i, 5));
    fkVelocityXYZ(i, 2) = (fkPositionXYZ(i+1, 2) - fkPositionXYZ(i, 2)) / (tjMatrixTotal(i + 1, 5) - tjMatrixTotal(i, 5));
    fkVelocityXYZ(i, 3) = (fkPositionXYZ(i+1, 3) - fkPositionXYZ(i, 3)) / (tjMatrixTotal(i + 1, 5) - tjMatrixTotal(i, 5));
end

% Calculating Acceleration
for i = 1:height(fkVelocityXYZ)-1
    fkAccelerationXYZ(i, 1) = (fkVelocityXYZ(i+1, 1) - fkVelocityXYZ(i, 1)) / (tjMatrixTotal(i + 1, 5) - tjMatrixTotal(i, 5));
    fkAccelerationXYZ(i, 2) = (fkVelocityXYZ(i+1, 2) - fkVelocityXYZ(i, 2)) / (tjMatrixTotal(i + 1, 5) - tjMatrixTotal(i, 5));
    fkAccelerationXYZ(i, 3) = (fkVelocityXYZ(i+1, 3) - fkVelocityXYZ(i, 3)) / (tjMatrixTotal(i + 1, 5) - tjMatrixTotal(i, 5));
end


% Plotting Total Position
figure(2);
plot(tjMatrixTotal(:, 5), fkPositionXYZ(:,1), 'LineWidth', 2, 'Color', 'r');
grid on;
hold on;
plot(tjMatrixTotal(:, 5), fkPositionXYZ(:,2), 'LineWidth', 2, 'Color', 'g');
plot(tjMatrixTotal(:, 5), fkPositionXYZ(:,3), 'LineWidth', 2, 'Color', 'b');
title('Part 7 Total Position vs. Time');
xlabel('Time (s)');
ylabel('Position (mm)');
legend('Position X', 'Position Y', 'Position Z');

% Plotting Total Velocity
tjMatrixTotal(height(tjMatrixTotal), :) = [];
figure(3);
plot(tjMatrixTotal(:, 5), fkVelocityXYZ(:,1), 'LineWidth', 2, 'Color', 'r');
grid on;
hold on;
plot(tjMatrixTotal(:, 5), fkVelocityXYZ(:,2), 'LineWidth', 2, 'Color', 'g');
plot(tjMatrixTotal(:, 5), fkVelocityXYZ(:,3), 'LineWidth', 2, 'Color', 'b');
title('Part 7 Total Velocity vs. Time');
xlabel('Time (s)');
ylabel('Velocity (mm/s)');
legend('Velocity X', 'Velocity Y', 'Velocity Z');

% Plotting Total Acceleration
tjMatrixTotal(height(tjMatrixTotal), :) = [];
figure(4);
plot(tjMatrixTotal(:, 5), fkAccelerationXYZ(:,1), 'LineWidth', 2, 'Color', 'r');
grid on;
hold on;
plot(tjMatrixTotal(:, 5), fkAccelerationXYZ(:,2), 'LineWidth', 2, 'Color', 'g');
plot(tjMatrixTotal(:, 5), fkAccelerationXYZ(:,3), 'LineWidth', 2, 'Color', 'b');
title('Part 7 Total Acceleration vs. Time');
xlabel('Time (s)');
ylabel('Acceleration (mm/s^2)');
legend('Acceleration X', 'Acceleration Y', 'Acceleration Z');

% Plotting 3D Path
figure(5);
plot3(fkPositionXYZ(:,1), fkPositionXYZ(:,2), fkPositionXYZ(:,3));
grid on;
title('Part 7 3D Path');
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
zlabel('Z Position (mm)');