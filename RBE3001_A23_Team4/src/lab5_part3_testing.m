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
% I = cam.getImage();
% 
% [BW,I2] = HSVMask(I);
% 
% for i = 0:1
%     BW = medfilt2(BW);
% end
% 
% BW(1:206,:) = 0;
% 
% s = regionprops(BW, 'centroid');
% 
% centroids = cat(1,s.Centroid);

% imshow(BW)
% hold on;
% plot(centroids(:,1), centroids(:,2),'b*')
% hold off

% locations = cam.getCentroids();
% 
% ball1Loc = cam.cam2Rob(locations(4,:));
% 
% ball1angles = robot.ik3001([ball1Loc(1), ball1Loc(2), 20, -pi/2]);
% 
% robot.interpolate_jp(2, [0 0 0 0]);
% pause(2);
% 
% robot.interpolate_jp(2, rad2deg(ball1angles));
% pause(2);

redLocation = cam.getCentroids('r');
orangeLocation = cam.getCentroids('o');
yellowLocation = cam.getCentroids('y');
greenLocation = cam.getCentroids('gn');
grayLocation = cam.getCentroids('gy');

I = cam.getImage();
figure;
imshow(I, 'InitialMagnification', 'fit');

hold on;
plot(redLocation(:,1), redLocation(:,2),'r*');
plot(orangeLocation(:,1), orangeLocation(:,2),'Color', '#FFA500', 'Marker','*');
plot(greenLocation(:,1), greenLocation(:,2),'g*');
plot(yellowLocation(:,1), yellowLocation(:,2),'y*');
plot(grayLocation(1,1), grayLocation(1,2), 'Color', '#808080', 'Marker','*');
hold off