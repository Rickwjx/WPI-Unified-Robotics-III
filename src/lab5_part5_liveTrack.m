clear;
clc;

%% Setup robot
robot = Robot();
trajPlanner = Traj_Planner();
model = Model();
% cam = Camera();
try
    load("camParams.mat");
    disp("Loaded Camera Parameters from camParams.mat");
catch exception
    disp("Could not find camParams.mat, creating new Camera object");
    cam = Camera();
    save("camParams.mat","cam");
    disp("Saved Camera Parameters to camParams.mat");
end

%% Writing Functions
robot.interpolate_jp(2, [0 0 0 0]);
pause(2);

camLocation = cam.getCentroids('gn');
taskSpaceLocation = cam.cam2Rob(camLocation);
jointSpaceLocation = rad2deg(robot.ik3001([taskSpaceLocation(1), taskSpaceLocation(2), 50, -pi/2]));

prevJointSpaceBall = jointSpaceLocation;
robot.interpolate_jp(2, jointSpaceLocation);
pause(2);

while true
    camLocation = cam.getCentroids('gn');
    if ~isempty(camLocation)
        taskSpaceLocation = cam.cam2Rob(camLocation);
        truePositions = [taskSpaceLocation(1), taskSpaceLocation(2), 50, -pi/2];

        currJointSpaceBall = rad2deg(robot.ik3001(truePositions));
    end

    if currJointSpaceBall ~= prevJointSpaceBall
        robot.interpolate_jp(0.1, currJointSpaceBall);

        prevJointSpaceBall = currJointSpaceBall;
    end
    pause(0.04);
end