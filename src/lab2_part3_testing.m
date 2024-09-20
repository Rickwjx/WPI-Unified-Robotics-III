%% Setup robot
travelTime = 2; % Defines the travel time
robot = Robot(); % Creates robot object
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

%% Program
robot.interpolate_jp(travelTime, [0 0 0 0]); % Write joints to zero position
pause(travelTime); % Wait for trajectory completion

% baseWayPoints = [-75]; %Define base waypoints
% dataArray = [];
% timeArray = [];
% i = 0;

desiredPositions = [23 -50 19 -54;
                    34 29 -51 -29;
                    10 -38 -69 44;
                    -66 20 37 -22;
                    50 -18 -27 59]
i = 1;
actualPositions = [];
expectedPositions = [];

view(3);
hold on;

for n = 1:5
    robot.interpolate_jp(travelTime, desiredPositions(i,:));
    pause(travelTime);

    actualMatrix = robot.setpoint_cp();
    actualPositions(i, 1) = actualMatrix(1, 4);
    actualPositions(i, 2) = actualMatrix(2, 4);
    actualPositions(i, 3) = actualMatrix(3, 4);
    
    desiredMatrix = robot.goal_cp();
    expectedPositions(i, 1) = desiredMatrix(1, 4);
    expectedPositions(i, 2) = desiredMatrix(2, 4);
    expectedPositions(i, 3) = desiredMatrix(3, 4);

    if i == 1
        actual1 = actualMatrix;
        expected1 = desiredMatrix;
    elseif i == 2
            actual2 = actualMatrix;
            expected2 = desiredMatrix;
    elseif i == 3
            actual3 = actualMatrix;
            expected3 = desiredMatrix;
    elseif i == 4
            actual4 = actualMatrix;
            expected4 = desiredMatrix;
    else
        actual5 = actualMatrix;
        expected5 = desiredMatrix;
    end

    
    plot3(actualPositions(i,1), actualPositions(i,2), actualPositions(i,3), ...
        expectedPositions(i,1), expectedPositions(i,2), expectedPositions(i,3), ...
        'Marker', 'o', 'LineWidth', 3);

    i = i + 1;
    
    pause(1);
end

grid on;
xlabel X-Axis;
ylabel Y-Axis;
zlabel Z-Axis;

% Symbolic Part 4
% syms ThetaStar1 ThetaStar2 ThetaStar3 ThetaStar4 Alpha1 Theta2 Theta3 Theta4 L0 L1 L2 L3 L4
% dhTable = [ThetaStar1 L1 0 Alpha1;
%            ThetaStar2+Theta2 0 L2 0;
%            ThetaStar3+Theta3 0 L3 0;
%            ThetaStar4+Theta4 0 L4 0];
% 
% T01 = robot.dh2mat([0 L0 0 0])
% T12 = robot.dh2mat(dhTable(1,:))
% T23 = robot.dh2mat(dhTable(2,:))
% T34 = robot.dh2mat(dhTable(3,:))
% T45 = robot.dh2mat(dhTable(4,:))
% 
% T05 = robot.dh2fk(dhTable, T01)