clear;
clc;
%% Setup robot
travelTime = 5; % Defines the travel time
% robot = Robot(); % Creates robot object
trajPlanner = Traj_Planner(); % Creates traj_Planner object which also creates robot object
robot = Robot();

% Get joint angles of each vertex of the triangle
homePosition = [0, 0 ,0 ,0];
point1Angles = robot.ik3001([200 200 115 -pi/4]) * (180 / pi);
point2Angles = robot.ik3001([175 0 390 pi/4]) * (180 / pi);
point3Angles = robot.ik3001([185 -107 131 0]) * (180 / pi);


% Moves to points without time interpoloation

% CHECK THIS WITH TA PLEASE (Two joints cannot move fast)
% robot.servo_jp(point1Angles);
% pause(2);
% 
% tic;
% robot.servo_jp(point2Angles);
% toc;
% pause(5);
% robot.servo_jp(point3Angles);
% pause(5);
% robot.servo_jp(point1Angles);
% pause(5);


% %Calculating Trajectory Coefficients for each joint moving from Home to Position 1
% trajCoef1 = trajPlanner.cubic_traj(0, 3, 0, 0, homePosition(1), point1Angles(1));
% trajCoef2 = trajPlanner.cubic_traj(0, 3, 0, 0, homePosition(2), point1Angles(2));
% trajCoef3 = trajPlanner.cubic_traj(0, 3, 0, 0, homePosition(3), point1Angles(3));
% trajCoef4 = trajPlanner.cubic_traj(0, 3, 0, 0, homePosition(4), point1Angles(4));
% 
% %Putting all the Trajectory Coefficients into a 4x4 matrix
% trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
% tjMatrix1 = trajPlanner.run_trajectory(trajCoef, 3);
% 
% %Converting Joint Angles into Task Space for Plotting
% for i = 1:height(tjMatrix1)
%     fkPosition1 = trajPlanner.fk3001(([tjMatrix1(i, 1), tjMatrix1(i, 2), tjMatrix1(i, 3), tjMatrix1(i, 4)] * pi / 180)');
%     fkPosition1XYZ(i, 1) = fkPosition1(1, 4);
%     fkPosition1XYZ(i, 2) = fkPosition1(2, 4);
%     fkPosition1XYZ(i, 3) = fkPosition1(3, 4);
% end
% 
% % Calculating Velocity
% for i = 1:height(fkPosition1XYZ)-1
%     fkVelocity1XYZ(i, 1) = (fkPosition1XYZ(i+1, 1) - fkPosition1XYZ(i, 1)) / (tjMatrix1(i + 1, 5) - tjMatrix1(i, 5));
%     fkVelocity1XYZ(i, 2) = (fkPosition1XYZ(i+1, 2) - fkPosition1XYZ(i, 2)) / (tjMatrix1(i + 1, 5) - tjMatrix1(i, 5));
%     fkVelocity1XYZ(i, 3) = (fkPosition1XYZ(i+1, 3) - fkPosition1XYZ(i, 3)) / (tjMatrix1(i + 1, 5) - tjMatrix1(i, 5));
% end
% 
% % Calculating Acceleration
% for i = 1:height(fkVelocity1XYZ)-1
%     fkAcceleration1XYZ(i, 1) = (fkVelocity1XYZ(i+1, 1) - fkVelocity1XYZ(i, 1)) / (tjMatrix1(i + 1, 5) - tjMatrix1(i, 5));
%     fkAcceleration1XYZ(i, 2) = (fkVelocity1XYZ(i+1, 2) - fkVelocity1XYZ(i, 2)) / (tjMatrix1(i + 1, 5) - tjMatrix1(i, 5));
%     fkAcceleration1XYZ(i, 3) = (fkVelocity1XYZ(i+1, 3) - fkVelocity1XYZ(i, 3)) / (tjMatrix1(i + 1, 5) - tjMatrix1(i, 5));
% end
% 
% % Plotting Movement from Home to Point 1
% figure(1);
% plot(tjMatrix1(:, 5), fkPosition1XYZ(:,1), 'LineWidth', 2, 'Color', 'r');
% hold on;
% plot(tjMatrix1(:, 5), fkPosition1XYZ(:,2), 'LineWidth', 2, 'Color', 'g');
% plot(tjMatrix1(:, 5), fkPosition1XYZ(:,3), 'LineWidth', 2, 'Color', 'b');
% 
% tjMatrix1(height(tjMatrix1), :) = [];
% figure(2);
% plot(tjMatrix1(:, 5), fkVelocity1XYZ(:,1), 'LineWidth', 2, 'Color', 'r');
% hold on;
% plot(tjMatrix1(:, 5), fkVelocity1XYZ(:,2), 'LineWidth', 2, 'Color', 'g');
% plot(tjMatrix1(:, 5), fkVelocity1XYZ(:,3), 'LineWidth', 2, 'Color', 'b');
% 
% tjMatrix1(height(tjMatrix1), :) = [];
% figure(3);
% plot(tjMatrix1(:, 5), fkAcceleration1XYZ(:,1), 'LineWidth', 2, 'Color', 'r');
% hold on;
% plot(tjMatrix1(:, 5), fkAcceleration1XYZ(:,2), 'LineWidth', 2, 'Color', 'g');
% plot(tjMatrix1(:, 5), fkAcceleration1XYZ(:,3), 'LineWidth', 2, 'Color', 'b');







%Calculating Trajectory Coefficients for each joint moving from Position 1 to Position 2
trajCoef1 = trajPlanner.cubic_traj(0, 4, 0, 0,  point1Angles(1), point2Angles(1));
trajCoef2 = trajPlanner.cubic_traj(0, 4, 0, 0,  point1Angles(2), point2Angles(2));
trajCoef3 = trajPlanner.cubic_traj(0, 4, 0, 0,  point1Angles(3), point2Angles(3));
trajCoef4 = trajPlanner.cubic_traj(0, 4, 0, 0,  point1Angles(4), point2Angles(4));

%Putting all the Trajectory Coefficients into a 4x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
%Running the Trajectory 
tjMatrix2 = trajPlanner.run_trajectory(trajCoef, 4, true);






%Calculating Trajectory Coefficients for each joint moving from Position 2 to Position 3
trajCoef1 = trajPlanner.cubic_traj(0, 5, 0, 0,  point2Angles(1), point3Angles(1));
trajCoef2 = trajPlanner.cubic_traj(0, 5, 0, 0,  point2Angles(2), point3Angles(2));
trajCoef3 = trajPlanner.cubic_traj(0, 5, 0, 0,  point2Angles(3), point3Angles(3));
trajCoef4 = trajPlanner.cubic_traj(0, 5, 0, 0,  point2Angles(4), point3Angles(4));

%Putting all the Trajectory Coefficients into a 4x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
tjMatrix3 = trajPlanner.run_trajectory(trajCoef, 5, true);



%Calculating Trajectory Coefficients for each joint moving from Position 3 to Position 1
trajCoef1 = trajPlanner.cubic_traj(0, 6, 0, 0,  point3Angles(1), point1Angles(1));
trajCoef2 = trajPlanner.cubic_traj(0, 6, 0, 0,  point3Angles(2), point1Angles(2));
trajCoef3 = trajPlanner.cubic_traj(0, 6, 0, 0,  point3Angles(3), point1Angles(3));
trajCoef4 = trajPlanner.cubic_traj(0, 6, 0, 0,  point3Angles(4), point1Angles(4));

%Putting all the Trajectory Coefficients into a 4x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
tjMatrix4 = trajPlanner.run_trajectory(trajCoef, 6, true);


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
title('Part 5 Joint Position vs. Time');
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
title('Part 5 Total Position vs. Time');
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
title('Part 5 Total Velocity vs. Time');
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
title('Part 5 Total Acceleration vs. Time');
xlabel('Time (s)');
ylabel('Acceleration (mm/s^2)');
legend('Acceleration X', 'Acceleration Y', 'Acceleration Z');

% Plotting 3D Path
figure(5);
plot3(fkPositionXYZ(:,1), fkPositionXYZ(:,2), fkPositionXYZ(:,3));
grid on;
title('Part 5 3D Path');
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
zlabel('Z Position (mm)');