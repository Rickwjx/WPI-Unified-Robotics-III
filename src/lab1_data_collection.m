%% Setup robot
travelTime = 2; % Defines the travel time
robot = Robot(); % Creates robot object
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

%% Program 

robot.interpolate_jp(travelTime, [0 0 0 0]); % Write joints to zero position
pause(travelTime); % Wait for trajectory completion

baseWayPoints = [-75]; %Define base waypoints
dataArray = [];
timeArray = [];
i = 0;

for baseWayPoint = baseWayPoints % Iterate though waypoints

    robot.servo_jp([baseWayPoint, 0, 0, -55])
    %robot.interpolate_jp(travelTime, [baseWayPoint, 0, 0, -55])

    tic;

    while toc < travelTime
        i = i + 1;
        dataArray(i, :) = robot.setpoint_js();
        timeArray(i, :) = toc;


    end


end

