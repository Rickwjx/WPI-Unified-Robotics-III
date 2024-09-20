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

object = 'Idle';

% Going to Home Position

robot.interpolate_jp(2, [0 0 0 0]);
pause(2);
noPicture = true;

while true
    switch object
        case 'Idle'
            clear redLocation orangeLocation yellowLocation greenLocation grayLocation;
            noPicture = true;
            redLocation = cam.getCentroids('r');
            orangeLocation = cam.getCentroids('o');
            yellowLocation = cam.getCentroids('y');
            greenLocation = cam.getCentroids('gn');
            grayLocation = cam.getCentroids('gy');

            if ~isempty(redLocation)
                tic;
                object = 'Red';
            elseif ~isempty(orangeLocation)
                tic;
                object = 'Orange';
            elseif ~isempty(yellowLocation)
                tic;
                object = 'Yellow';
            elseif ~isempty(greenLocation)
                tic;
                object = 'Green';
%             elseif ~isempty(grayLocation)
%                 tic;
%                 object = 'Gray';
            end
    
        case 'Red'
            if noPicture
                offsetPositions = cam.cam2Rob(redLocation(1, :));
                truePositions = cam.findTruePos(offsetPositions(1:2));

                taskSpaceBallAbove = [truePositions(1), truePositions(2), 80, -pi/2];
                jointSpaceBallAbove = rad2deg(robot.ik3001(taskSpaceBallAbove));
                taskSpaceAtBall = [truePositions(1), truePositions(2), 15, -pi/2];
                jointSpaceAtBall = rad2deg(robot.ik3001(taskSpaceAtBall));

                jointSpaceBox = rad2deg(robot.ik3001([215, 200, 60, -pi/4]));
                noPicture = false;
            end

            if toc > 1
                % Open Gripper
                robot.writeGripper(true);
                pause(0.5);
                % Moving to Above Red Ball
                robot.run_trajectory_point([0 0 0 0], jointSpaceBallAbove, 3, true);
                pause(0.5);
                % Moving down to Red Ball
                robot.run_trajectory_point(taskSpaceBallAbove, taskSpaceAtBall, 2, false);
                pause(0.5);
                % Close Gripper
                robot.writeGripper(false);
                pause(0.5);
    
                % Moving up with Ball
                robot.run_trajectory_point(taskSpaceAtBall, taskSpaceBallAbove, 2, false);
                pause(0.5);
                % Moving to Sorting Location
                robot.run_trajectory_point(jointSpaceBallAbove, jointSpaceBox, 3, true);
                pause(0.5);
                % Open Gripper
                robot.writeGripper(true);
                pause(0.5);
    
                % Move back to home position
                robot.run_trajectory_point(jointSpaceBox, [0, 0 ,0 ,0], 3, true);
                pause(0.5);
    
                object = 'Idle';
            end
    
        case 'Orange'
            if noPicture
                offsetPositions = cam.cam2Rob(orangeLocation(1, :));
                truePositions = cam.findTruePos(offsetPositions(1:2));

                taskSpaceBallAbove = [truePositions(1), truePositions(2), 80, -pi/2];
                jointSpaceBallAbove = rad2deg(robot.ik3001(taskSpaceBallAbove));
                taskSpaceAtBall = [truePositions(1), truePositions(2), 15, -pi/2];
                jointSpaceAtBall = rad2deg(robot.ik3001(taskSpaceAtBall));

                jointSpaceBox = rad2deg(robot.ik3001([70, -200, 60, -pi/4]));
                noPicture = false;
            end

            if toc > 1
                % Open Gripper
                robot.writeGripper(true);
                pause(0.5);
                % Moving to Above Red Ball
                robot.run_trajectory_point([0 0 0 0], jointSpaceBallAbove, 3, true);
                pause(0.5);
                % Moving down to Red Ball
                robot.run_trajectory_point(taskSpaceBallAbove, taskSpaceAtBall, 2, false);
                pause(0.5);
                % Close Gripper
                robot.writeGripper(false);
                pause(0.5);
    
                % Moving up with Ball
                robot.run_trajectory_point(taskSpaceAtBall, taskSpaceBallAbove, 2, false);
                pause(0.5);
                % Moving to Sorting Location
                robot.run_trajectory_point(jointSpaceBallAbove, jointSpaceBox, 3, true);
                pause(0.5);
                % Open Gripper
                robot.writeGripper(true);
                pause(0.5);
    
                % Move back to home position
                robot.run_trajectory_point(jointSpaceBox, [0, 0 ,0 ,0], 3, true);
                pause(0.5);
    
                object = 'Idle';
            end
    
        case 'Yellow'
            if noPicture
                offsetPositions = cam.cam2Rob(yellowLocation(1, :));
                truePositions = cam.findTruePos(offsetPositions(1:2));

                taskSpaceBallAbove = [truePositions(1), truePositions(2), 80, -pi/2];
                jointSpaceBallAbove = rad2deg(robot.ik3001(taskSpaceBallAbove));
                taskSpaceAtBall = [truePositions(1), truePositions(2), 15, -pi/2];
                jointSpaceAtBall = rad2deg(robot.ik3001(taskSpaceAtBall));

                jointSpaceBox = rad2deg(robot.ik3001([215, -200, 60, -pi/4]));
                noPicture = false;
            end

            if toc > 1
                % Open Gripper
                robot.writeGripper(true);
                pause(0.5);
                % Moving to Above Red Ball
                robot.run_trajectory_point([0 0 0 0], jointSpaceBallAbove, 3, true);
                pause(0.5);
                % Moving down to Red Ball
                robot.run_trajectory_point(taskSpaceBallAbove, taskSpaceAtBall, 2, false);
                pause(0.5);
                % Close Gripper
                robot.writeGripper(false);
                pause(0.5);
    
                % Moving up with Ball
                robot.run_trajectory_point(taskSpaceAtBall, taskSpaceBallAbove, 2, false);
                pause(0.5);
                % Moving to Sorting Location
                robot.run_trajectory_point(jointSpaceBallAbove, jointSpaceBox, 3, true);
                pause(0.5);
                % Open Gripper
                robot.writeGripper(true);
                pause(0.5);
    
                % Move back to home position
                robot.run_trajectory_point(jointSpaceBox, [0, 0 ,0 ,0], 3, true);
                pause(0.5);
    
                object = 'Idle';
            end
    
        case 'Green'
            if noPicture
                offsetPositions = cam.cam2Rob(greenLocation(1, :));
                truePositions = cam.findTruePos(offsetPositions(1:2));

                taskSpaceBallAbove = [truePositions(1), truePositions(2), 80, -pi/2];
                jointSpaceBallAbove = rad2deg(robot.ik3001(taskSpaceBallAbove));
                taskSpaceAtBall = [truePositions(1), truePositions(2), 15, -pi/2];
                jointSpaceAtBall = rad2deg(robot.ik3001(taskSpaceAtBall));

                jointSpaceBox = rad2deg(robot.ik3001([70, 200, 60, -pi/4]));
                noPicture = false;
            end

            if toc > 1
                % Open Gripper
                robot.writeGripper(true);
                pause(0.5);
                % Moving to Above Red Ball
                robot.run_trajectory_point([0 0 0 0], jointSpaceBallAbove, 3, true);
                pause(0.5);
                % Moving down to Red Ball
                robot.run_trajectory_point(taskSpaceBallAbove, taskSpaceAtBall, 2, false);
                pause(0.5);
                % Close Gripper
                robot.writeGripper(false);
                pause(0.5);
    
                % Moving up with Ball
                robot.run_trajectory_point(taskSpaceAtBall, taskSpaceBallAbove, 2, false);
                pause(0.5);
                % Moving to Sorting Location
                robot.run_trajectory_point(jointSpaceBallAbove, jointSpaceBox, 3, true);
                pause(0.5);
                % Open Gripper
                robot.writeGripper(true);
                pause(0.5);
    
                % Move back to home position
                robot.run_trajectory_point(jointSpaceBox, [0, 0 ,0 ,0], 3, true);
                pause(0.5);
    
                object = 'Idle';
            end
    
        case 'Gray'
            if noPicture
                offsetPositions = cam.cam2Rob(grayLocation(1, :));
                truePositions = cam.findTruePos(offsetPositions(1:2));

                taskSpaceBallAbove = [truePositions(1), truePositions(2), 80, -pi/2];
                jointSpaceBallAbove = rad2deg(robot.ik3001(taskSpaceBallAbove));
                taskSpaceAtBall = [truePositions(1), truePositions(2), 15, -pi/2];
                jointSpaceAtBall = rad2deg(robot.ik3001(taskSpaceAtBall));

                jointSpaceBox = rad2deg(robot.ik3001([140, -200, 60, -pi/4]));
                noPicture = false;
            end

            if toc > 1
                % Open Gripper
                robot.writeGripper(true);
                pause(0.5);
                % Moving to Above Red Ball
                robot.run_trajectory_point([0 0 0 0], jointSpaceBallAbove, 3, true);
                pause(0.5);
                % Moving down to Red Ball
                robot.run_trajectory_point(taskSpaceBallAbove, taskSpaceAtBall, 2, false);
                pause(0.5);
                % Close Gripper
                robot.writeGripper(false);
                pause(0.5);
    
                % Moving up with Ball
                robot.run_trajectory_point(taskSpaceAtBall, taskSpaceBallAbove, 2, false);
                pause(0.5);
                % Moving to Sorting Location
                robot.run_trajectory_point(jointSpaceBallAbove, jointSpaceBox, 3, true);
                pause(0.5);
                % Open Gripper
                robot.writeGripper(true);
                pause(0.5);
    
                % Move back to home position
                robot.run_trajectory_point(jointSpaceBox, [0, 0 ,0 ,0], 3, true);
                pause(0.5);
    
                object = 'Idle';
            end

    end % End switch
end