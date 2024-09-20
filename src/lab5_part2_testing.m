clc;
clear;
clc;
robot = Robot();
model = Model();
cam = Camera();    

I = cam.getImage();
    imshow(I);
