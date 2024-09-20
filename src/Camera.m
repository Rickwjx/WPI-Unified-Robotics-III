classdef Camera < handle
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties        
        % Properties
        params;     % Camera Parameters
        cam;        % Webcam Object
        cam_pose;   % Camera Pose (transformation matrix)
        cam_IS;     % Camera Intrinsics
        cam_R;      % Camera Rotation Matrix
        cam_T;      % Camera Translation Vector
        cam_TForm   % Camera Rigid 3D TForm
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            % make sure that the webcam can see the whole checkerboard by
            % running webcam(2).preview in the Command Window
            self.cam = webcam(2); % Get camera object
            self.params = self.calibrate(); % Run Calibration Function
            [self.cam_IS, self.cam_pose] = self.calculateCameraPos();
        end

        function tForm = getTForm(self)
            tForm = self.cam_TForm;
        end

        function cam_pose = getCameraPose(self)
            cam_pose = self.cam_pose;
        end

        function cam_IS = getCameraIntrinsics(self)
            cam_IS = self.cam_IS;
        end

        function cam_R = getRotationMatrix(self)
            cam_R = self.cam_R;
        end

        function cam_T = getTranslationVector(self)
            cam_T = self.cam_T;
        end

        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end
      
        function params = calibrate(self)
            % CALIBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camera_calibration.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                disp("Calibrating");
                camcalib; % Change this if you are using a different calibration script
                params = cameraParams;
                disp("Camera calibration complete!");
            catch exception
                msg = getReport(exception);
                disp(msg)
                disp("No camera calibration file found. Plese run camera calibration");
            end          
        end
        
        % Returns an undistorted camera image
        function img = getImage(self)
            raw_img =  snapshot(self.cam);
            [img, new_origin] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
        end

        
        function [newIs, pose] = calculateCameraPos(self)  % DO NOT USE
            % calculateCameraPos Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, newIs] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img, 'PartialDetections', false);
            % 4. Compute transformation
            self.params.WorldPoints = self.params.WorldPoints(self.params.WorldPoints(:, 2) <= (boardSize(1)-1)*25, :);
            worldPointSize = size(self.params.WorldPoints);
            imagePointSize = size(imagePoints);
            fprintf("World Points is %d x %d\n", worldPointSize(1), worldPointSize(2));
            fprintf("Image Points is %d x %d\n", imagePointSize(1), imagePointSize(2));
            fprintf("The checkerboard is %d squares long x %d squares wide\n", boardSize(1), boardSize(2));

            % 4. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, newIs);

            self.cam_R = R;
            self.cam_T = t;
            self.cam_TForm = rigid3d([ self.cam_R, zeros(3,1); self.cam_T, 1 ]);
            
            pose = [   R,    t';
                    0, 0, 0, 1];
        end

        % Takes in the coordinates from the image and converts it to the
        % task space coordinates of the robot
        function positions = cam2Rob(self, camPos)
            [pose] = self.getCameraPose();
            % disp("T_img_check");
            % disp(pose);
            R = pose(1:3, 1:3);
            t = pose(1:3, 4);
            pos = [camPos(1), camPos(2)]; %x, y pixel position of a selected point on the image
            % disp(pos);

            % Convert pixel position to world coordinates
            worldPt = pointsToWorld(self.getCameraIntrinsics(), R, t, pos);
            % disp(worldPt);
            R_0_checker = [0  1.15  0; 1.15  0  0; 0  0 -1];
            t_0_checker = [113; -95; 0];
            T_0_check = [R_0_checker, t_0_checker;zeros(1,3), 1];
            r_pos = inv(T_0_check) * [worldPt'; 0; 1];

            positions = r_pos';
        end

        % Takes an image of the board and returns the location of the
        % centroids of the balls on the board
        function centroids = getCentroids(self, Color)
            I = self.getImage();

            % Masking out the background
            checkVertices = [313.0000, 206.0000;
                             765.0000, 214.0000;
                             882.0000, 411.0000;
                             181.0000, 408.0000];
            BWMask = ~poly2mask(checkVertices(:, 1)', checkVertices(:, 2)', size(I, 1), size(I, 2));
            BWMask = repmat(BWMask, [1 1 size(I, 3)]);
            mask = I;
            mask(BWMask) = 0;
%             imshow(mask)

            if Color == 'r'
                [BW,~] = redMask(mask);
            elseif Color == 'gn'
                [BW,~] = greenMask(mask);
            elseif Color == 'o'
                [BW,~] = orangeMask(mask);
            elseif Color == 'y'
                [BW,~] = yellowMask(mask);
            elseif Color == 'gy'
                [BW, ~] = grayMask(mask);
            end

            BW = imfill(BW, 'holes');
            
            for i = 0:4
                BW = medfilt2(BW);
            end
            
            s = regionprops(BW, 'centroid', 'Area');

            centroids = [];


            for i = 1:length(s)
                if s(i).Area >= 1000
                    centroids = [centroids; s(i).Centroid];
                end
            end

            
%             imshow(BW)
%             hold on;
%             plot(centroids(:,1), centroids(:,2),'b*')
%             hold off
        end

        % Takes in the positions of the task space of the centroids and
        % takes care of the offset from the camera to output the correct
        % location of the balls in the task space
        function Position = findTruePos(self, Positions)
            Xpos = 358 - Positions(1);
            Ypos = Positions(2);
            % distance = sqrt(Xpos^2 + Ypos^2);
            
            ratio = 182.5 / 9.5;

            offsetX = Xpos / ratio;
            offsetY = Ypos / ratio;

            disp(Xpos);
            disp(Ypos);
            disp(offsetY);

%             Position = [Positions(1) - offsetX, Positions(2) + offsetY];
            Position = [Positions(1), Positions(2)];
            
        end

    end % end Methods
end