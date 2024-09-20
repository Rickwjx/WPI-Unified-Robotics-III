clear;
clc;

%% Setup robot
robot = Robot();
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
% function Position = findTruePos(self, Positions)
%     Xpos = 358 - Positions(1);
%     Ypos = Positions(2);
%     distance = sqrt(Xpos^2 + Ypos^2);
% 
%     trueX = Xpos + 12.5 * Xpos/distance;
%     trueY = Ypos + 12.5 * Ypos/distance;
% 
%     Position = [trueX, trueY];
% end

locations = cam.getCentroids('r');

offsetPositions = cam.cam2Rob(locations(1, :));

truePositions = cam.findTruePos(offsetPositions(1:2));

robot.writeGripper(true);
robot.interpolate_jp(2, [0 0 0 0]);
pause(2);

robot.interpolate_jp(2, rad2deg(robot.ik3001([truePositions(1), truePositions(2), 15, -pi/2])));
pause(2);
robot.writeGripper(false);

robot.interpolate_jp(2, [0 0 0 0]);
pause(2);