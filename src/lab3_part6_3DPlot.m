clear;
clc;
%% Setup robot
travelTime = 5; % Defines the travel time
trajPlanner = Traj_Planner(); % Creates model object which also creates robot object
robot = Robot();

dataArray = [];
timeArray = [];
i = 0;


% Interpolate_jp Movement
triPos = [200 200 115 -pi/4; 
          175 0 390 pi/4;
          185 -107 131 0];

% robot.interpolate_jp(travelTime, robot.ik3001(triPos(1, :)) * 180/pi);
% pause(travelTime);

% Moving from 1 to 2
robot.interpolate_jp(travelTime, robot.ik3001(triPos(2, :)) * 180/pi);
tic
while toc < travelTime
    i = i + 1;
    transformationMatrix = robot.fk3001((robot.setpoint_js()*pi/180)');
    positions(i, 1) = transformationMatrix(1, 4);
    positions(i, 2) = transformationMatrix(2, 4);
    positions(i, 3) = transformationMatrix(3, 4);
end

% Moving from 2 to 3
robot.interpolate_jp(travelTime, robot.ik3001(triPos(3, :)) * 180/pi);
tic
while toc < travelTime
    i = i + 1;
    transformationMatrix = robot.fk3001((robot.setpoint_js()*pi/180)');
    positions(i, 1) = transformationMatrix(1, 4);
    positions(i, 2) = transformationMatrix(2, 4);
    positions(i, 3) = transformationMatrix(3, 4);
end

% Moving from 3 to 1
robot.interpolate_jp(travelTime, robot.ik3001(triPos(1, :)) * 180/pi);
tic
while toc < travelTime
    i = i + 1;
    transformationMatrix = robot.fk3001((robot.setpoint_js()*pi/180)');
    positions(i, 1) = transformationMatrix(1, 4);
    positions(i, 2) = transformationMatrix(2, 4);
    positions(i, 3) = transformationMatrix(3, 4);
end




% Joint Space Trajectory Movement
point1Angles = robot.ik3001([200 200 115 -pi/4]) * (180 / pi);
point2Angles = robot.ik3001([175 0 390 pi/4]) * (180 / pi);
point3Angles = robot.ik3001([185 -107 131 0]) * (180 / pi);

% Calculating Trajectory Coefficients for each joint moving from Position 1 to Position 2
trajCoef1 = trajPlanner.cubic_traj(0, 4, 0, 0,  point1Angles(1), point2Angles(1));
trajCoef2 = trajPlanner.cubic_traj(0, 4, 0, 0,  point1Angles(2), point2Angles(2));
trajCoef3 = trajPlanner.cubic_traj(0, 4, 0, 0,  point1Angles(3), point2Angles(3));
trajCoef4 = trajPlanner.cubic_traj(0, 4, 0, 0,  point1Angles(4), point2Angles(4));

% Putting all the Trajectory Coefficients into a 4x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
% Running the Trajectory 
tjMatrix2 = trajPlanner.run_trajectory(trajCoef, 4, true);


% Calculating Trajectory Coefficients for each joint moving from Position 2 to Position 3
trajCoef1 = trajPlanner.cubic_traj(0, 5, 0, 0,  point2Angles(1), point3Angles(1));
trajCoef2 = trajPlanner.cubic_traj(0, 5, 0, 0,  point2Angles(2), point3Angles(2));
trajCoef3 = trajPlanner.cubic_traj(0, 5, 0, 0,  point2Angles(3), point3Angles(3));
trajCoef4 = trajPlanner.cubic_traj(0, 5, 0, 0,  point2Angles(4), point3Angles(4));

% Putting all the Trajectory Coefficients into a 4x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
tjMatrix3 = trajPlanner.run_trajectory(trajCoef, 5, true);


% Calculating Trajectory Coefficients for each joint moving from Position 3 to Position 1
trajCoef1 = trajPlanner.cubic_traj(0, 6, 0, 0,  point3Angles(1), point1Angles(1));
trajCoef2 = trajPlanner.cubic_traj(0, 6, 0, 0,  point3Angles(2), point1Angles(2));
trajCoef3 = trajPlanner.cubic_traj(0, 6, 0, 0,  point3Angles(3), point1Angles(3));
trajCoef4 = trajPlanner.cubic_traj(0, 6, 0, 0,  point3Angles(4), point1Angles(4));

% Putting all the Trajectory Coefficients into a 4x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
tjMatrix4 = trajPlanner.run_trajectory(trajCoef, 6, true);

% Combining Joint Angles
tjMatrixTotal = vertcat(tjMatrix2, tjMatrix3, tjMatrix4);

% Converting Joint Angles into Task Space for Plotting
for i = 1:height(tjMatrixTotal)
    fkPosition = trajPlanner.fk3001(([tjMatrixTotal(i, 1), tjMatrixTotal(i, 2), tjMatrixTotal(i, 3), tjMatrixTotal(i, 4)] * pi / 180)');
    fkPositionXYZ(i, 1) = fkPosition(1, 4);
    fkPositionXYZ(i, 2) = fkPosition(2, 4);
    fkPositionXYZ(i, 3) = fkPosition(3, 4);
end




% Task Space Trajectory Movement
point1 = [200 200 115 -pi/4];
point2 = [175 0 390 pi/4];
point3 = [185 -107 131 0];

%Calculating Trajectory Coefficients for each joint moving from Position 1 to Position 2
trajCoef1 = trajPlanner.cubic_traj(0, 4, 0, 0,  point1(1), point2(1));
trajCoef2 = trajPlanner.cubic_traj(0, 4, 0, 0,  point1(2), point2(2));
trajCoef3 = trajPlanner.cubic_traj(0, 4, 0, 0,  point1(3), point2(3));
trajCoef4 = trajPlanner.cubic_traj(0, 4, 0, 0,  point1(4), point2(4));

%Putting all the Trajectory Coefficients into a 4x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
%Running the Trajectory 
tjMatrix2 = trajPlanner.run_trajectory(trajCoef, 4, false);

%Calculating Trajectory Coefficients for each joint moving from Position 2 to Position 3
trajCoef1 = trajPlanner.cubic_traj(0, 4, 0, 0,  point2(1), point3(1));
trajCoef2 = trajPlanner.cubic_traj(0, 4, 0, 0,  point2(2), point3(2));
trajCoef3 = trajPlanner.cubic_traj(0, 4, 0, 0,  point2(3), point3(3));
trajCoef4 = trajPlanner.cubic_traj(0, 4, 0, 0,  point2(4), point3(4));

%Putting all the Trajectory Coefficients into a 4x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
%Running the Trajectory 
tjMatrix3 = trajPlanner.run_trajectory(trajCoef, 4, false);

%Calculating Trajectory Coefficients for each joint moving from Position 3 to Position 1
trajCoef1 = trajPlanner.cubic_traj(0, 4, 0, 0,  point3(1), point1(1));
trajCoef2 = trajPlanner.cubic_traj(0, 4, 0, 0,  point3(2), point1(2));
trajCoef3 = trajPlanner.cubic_traj(0, 4, 0, 0,  point3(3), point1(3));
trajCoef4 = trajPlanner.cubic_traj(0, 4, 0, 0,  point3(4), point1(4));

%Putting all the Trajectory Coefficients into a 4x4 matrix
trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
%Running the Trajectory 
tjMatrix4 = trajPlanner.run_trajectory(trajCoef, 4, false);

% Combining Joint Angles
tjMatrixTotal2 = vertcat(tjMatrix2, tjMatrix3, tjMatrix4);

% Converting Joint Angles into Task Space for Plotting
for i = 1:height(tjMatrixTotal2)
    fkPosition = trajPlanner.fk3001(([tjMatrixTotal2(i, 1), tjMatrixTotal2(i, 2), tjMatrixTotal2(i, 3), tjMatrixTotal2(i, 4)] * pi / 180)');
    fkPositionXYZ2(i, 1) = fkPosition(1, 4);
    fkPositionXYZ2(i, 2) = fkPosition(2, 4);
    fkPositionXYZ2(i, 3) = fkPosition(3, 4);
end



% Measured Path in 3D Space
figure(1);
view(3);
plot3(positions(:,1), positions(:,2), positions(:,3), 'LineWidth', 2, 'Color', 'r');
hold on;
plot3(fkPositionXYZ(:,1), fkPositionXYZ(:,2), fkPositionXYZ(:,3), 'LineWidth', 2, 'Color', 'g');
plot3(fkPositionXYZ2(:,1), fkPositionXYZ2(:,2), fkPositionXYZ2(:,3), 'LineWidth', 2, 'Color', 'b');
grid on;
title('Path in 3D Space with Different Trajectory Modes');
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
zlabel('Z Position (mm)');
legend('Positions with Joint Interpolation', 'Positions with Joint Space Trajectory Planning', ...
    'Positions with Task Space Trajectory Planning');

