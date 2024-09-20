
clear;
clc;
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

try
    [pose] = cam.getCameraPose();
    disp("T_img_check");
    disp(pose);
    R = pose(1:3, 1:3);
    t = pose(1:3, 4);
%     pos = [346, 256]; %x, y pixel position of a selected point on the image
    pos = [732, 265];
%     pos = [298, 347];
%     pos = [775, 350];
    disp(pos);
    % convert pixel position to world coordinates
    worldPt = pointsToWorld(cam.getCameraIntrinsics(), R, t, pos);
    disp(worldPt);
    R_0_checker = [0  1.15  0; 1.05  0  0; 0  0 -1];
    t_0_checker = [105; -80; 0];
    T_0_check = [R_0_checker, t_0_checker;zeros(1,3), 1];
    r_pos = inv(T_0_check) * [worldPt'; 0; 1];

%     r_pos = centers_to_positions(r_pos(1:2)');
    disp(r_pos);

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

robot.interpolate_jp(2, [0 0 0 0]);
pause(2);

robot.interpolate_jp(2, rad2deg(robot.ik3001([r_pos(1), r_pos(2), 20, -pi/2])));
pause(2);