% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        goals; % Stores end of Motion Joint Positions
    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);

            % Robot Dimensions
            self.mDim = [96.326, 130.23, 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
            self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);

        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
      
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;

            disp("time")
            disp(time_ms)
            disp("acc time")
            disp(acc_time_ms)

            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-25);
            else
                self.gripper.writePosition(35);
            end
        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the
        % joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end

        %takes a 1x4 array of joint values in degrees to be sent directly to the actuators and
        %bypasses interpolation
        %goals [1x4 double] - goal for the arm to reach
        function servo_jp(self, goals)
            
            self.writeTime(0.01);
            self.goals = goals;
            self.writeJoints(goals);

        end

        %Takes a 1x4 array of joint values and an interpolation time in ms to get there
        %time - time interpolation
        %goals [1x4 double] - goal for the arm to reach.
        function interpolate_jp(self, time, goals)
            
            self.goals = goals;
            self.writeTime(time);
            self.writeJoints(goals);
           

        end

        % Allows the user to obtain joint position data, joint velocity
        % data, or both
        function [measuredData] = measured_js(self, GETPOS, GETVEL)
            z = zeros(1, 4);
            readings = self.getJointsReadings();

            if GETPOS == true && GETVEL == true
                measuredData(1:2, :) = readings(1:2, :);
            elseif GETPOS == true
                measuredData = readings(1, :);
            elseif GETVEL == true
                measuredData = readings(2, :);
            else
                measuredData = z;
            end

        end

        % Returns the current joint set point positions in degrees
        function [setpoint] = setpoint_js(self)

            readings = self.getJointsReadings();
            setpoint = readings(1,:);

        end

        % Returns the goals variable 
        function [endPosition] = goal_js(self)
            endPosition = self.goals;
        end
        
        % Takes in an nx4 array corresponding to the n rows of the full PH
        % parameter table together with T01, and generates a corresponding
        % symbolic 4x4 homogeneous transformation matrix for the composite
        % transformation matrix.
        function [tMatrix] = dh2fk(self, dhParam, t01Mat)
            tMatrix = t01Mat;
            for n = 1:height(dhParam)
                tMatrix = tMatrix * self.dh2mat(dhParam(n, :));
            end
        end

        %Takes in DH parameters and returns homogenous transformation matrix
        %Angle values must be in radians!
        function [transformationMatrix] = dh2mat(self, dhParam)

            transformationMatrix = [cos(dhParam(1)), -1*sin(dhParam(1))*cos(dhParam(4)), sin(dhParam(1))*sin(dhParam(4)), dhParam(3)*cos(dhParam(1));
                sin(dhParam(1)), cos(dhParam(1))*cos(dhParam(4)), -1*cos(dhParam(1))*sin(dhParam(4)), dhParam(3)*sin(dhParam(1));
                0, sin(dhParam(4)), cos(dhParam(4)), dhParam(2);
                0, 0, 0, 1];

        end

        % Takes in n joint values and returns a 4x4 homogeneous
        % transformation matrix representing the position and orientation
        % of the tip frame with respect to the base frame
        function [transMatrix] = fk3001(self, joint)
            % Link Lengths (mm)
            L1 = 96.326 - 36.076;
            L2 = sqrt(128^2 + 24^2);
            L3 = 124;
            L4 = 133.4;
            % Theta (Radians)
            alpha1 = -pi/2;
            theta2 = atan(24 / 128) - (pi/2);
            theta3 = (pi/2) - atan(24 / 128);
            theta4 = 0;

            % DH Table of RBE 3001 Robot at zero position
            modifiedMatrix = [0,   L1,   0,   alpha1;
                              theta2,   0,   L2,   0;
                              theta3,   0,   L3,   0;
                              theta4,   0,   L4,   0];

            for n = 1:height(joint)
                modifiedMatrix(n, 1) = modifiedMatrix(n,1)+joint(n);
            end

            % T0-1 Transformation Matrix
            T0Matrix = self.dh2mat([0 36.076 0 0]);
            
            transMatrix = self.dh2fk(modifiedMatrix, T0Matrix);
        end

        % Takes data from measured_js() and returns a 4x4 homogeneous
        % transformation matrix based on the current joint positions.
        function [measuredMatrix] = measured_cp(self)
            measuredMatrix = self.fk3001((self.measured_js(true, false)*pi/180)');
        end

        % Takes data from setpoint_js() and returns a 4x4 homogeneous
        % transformation matrix based upon the current joint set point
        % positions.
        function [setpointMatrix] = setpoint_cp(self)
            setpointMatrix = self.fk3001((self.setpoint_js()*pi/180)');
        end

        % Takes data from goal_js() and returns a 4x4 homogeneous
        % transformation matrix based upon the commanded end position joint
        % set point positions.
        function [goalMatrix] = goal_cp(self)
            goalMatrix = self.fk3001((self.goal_js()*pi/180)');
        end  

        % Takes in 1x4 matrix containing coordinates and orientation of ee
        % Returns 1x4 matrix containing joint angles
        function [jointAngles] = ik3001(self, eePos)

            %Inputted ee parameters
            xPos = eePos(1);
            yPos = eePos(2);
            zPos = eePos(3);
            alpha = eePos(4);

            %Link lengths
            L0 = 36.076;
            L1 = 96.326 - L0;
            L2 = sqrt(128^2 + 24^2);
            L3 = 124;
            L4 = 133.4;

            r = sqrt(xPos^2 + yPos^2);

            theta1Pos = atan2(sqrt(1 - (xPos/r)^2), (xPos/r));
            theta1Neg = atan2(-1*sqrt(1 - (xPos/r)^2), (xPos/r));
            theta1 = 0;

            % Checks to see if theta 1 is a value number within robot's
            % workspace and to choose the right quadrant for theta1 to go
            % into
            if theta1Pos < pi/2 && theta1Neg > -1*pi/2
                if xPos > 0 && yPos > 0
                    theta1 = theta1Pos;
                else
                    theta1 = theta1Neg;
                end
            else
                error('Unable to find valid theta1 value.');
            end


            %Location of ee joint
            X4 = xPos - cos(theta1) * cos(alpha) * L4;
            Y4 = yPos - sin(theta1) * cos(alpha) * L4;
            Z4 = zPos - L4 * sin(alpha);            

            %Theta3 Caclulations  -4.2189e+0
            F = -1*(L2^2 + L3^2 - ((Z4-L1-L0)^2 + X4^2 + Y4^2))/(2*L2*L3);
            G = sqrt(1 - F^2);

            disp(F);
            disp(G);

            theta3Pos = atan2(G, F);
            theta3Neg = atan2(-1*G, F);
            theta3 = 0;

            if theta3Pos - ((pi/2) - atan(24 / 128)) > pi/2 && theta3Neg + ((pi/2) - atan(24 / 128)) < -1*pi/2
                error('Unable to find valid theta3 value.');
            else
                theta3 = theta3Pos;
            end

            % Theta2 Calculations
            H = (L2^2 + ((Z4 - L1 - L0)^2 + X4^2 + Y4^2) - L3^2) / (2*L2 * sqrt((Z4 - L1 - L0)^2 + X4^2 + Y4^2));
            I = sqrt(1 - H^2);

            phi = atan2(I, H);
            

            J = ((X4^2 + Y4^2) + (X4^2 + Y4^2 + (Z4 - L1 - L0)^2) - (Z4 - L1 - L0)^2) / (2 * sqrt(X4^2 + Y4^2) * sqrt(X4^2 + Y4^2 + (Z4 - L1 - L0)^2));
            K = sqrt(1 - J^2);

            mu = atan2(K, J);
            theta2 = 0;

            if Z4 > L1+L0
                theta2 = mu + phi;
            else
                theta2 = mu - phi;
            end

            theta4 = alpha - theta2 + theta3;
            


            jointAngles = [theta1, (pi/2 - atan(24/128)) - theta2, theta3 - ((pi/2) - atan(24 / 128)), -theta4];
            fkSolution = self.fk3001(jointAngles');
            if round(fkSolution(1,4),2) ~= round(xPos,2) || round(fkSolution(2,4),2) ~= round(yPos,2) || round(fkSolution(3,4),2) ~= round(zPos,2)
                error('Desired pose not possible');
            end

        end

        % Takes in trajector coefficients (4x4 Matrix) with the
        % coefficients of all 4 joints and the total time the trajector 
        % should take, which is the same duration used in cubic_traj.
        % It then calculates the joint poses and moves the robot while
        % saving time and joint angle data.
        % It returns the saved matrix data.
        function [tjMatrix] = run_trajectory(self, trajCoef, travelTime, jointSpace)
            tic;
            currentTime = 0;
            i = 0;
            while currentTime < travelTime
                i = i + 1;
                currentTime = toc;
                
                if height(trajCoef) == 4
                    for n = 1:width(trajCoef)
                        trajectory(n) = trajCoef(1,n) + trajCoef(2,n)*currentTime + trajCoef(3,n)*currentTime^2 + trajCoef(4,n)*currentTime^3;
                    end
                elseif height(trajCoef) == 6
                    for n = 1:width(trajCoef)
                        trajectory(n) = trajCoef(1,n) + trajCoef(2,n)*currentTime + trajCoef(3,n)*currentTime^2 + trajCoef(4,n)*currentTime^3 + ...
                        trajCoef(5, n)*currentTime^4 + trajCoef(6, n)*currentTime^5;
                    end 
                end
                
                if jointSpace == true
                    jointPosition = [trajectory(1), trajectory(2), trajectory(3), trajectory(4)];
                elseif jointSpace == false
                    taskPosition = [trajectory(1), trajectory(2), trajectory(3), trajectory(4)];
                    jointPosition = rad2deg(self.ik3001(taskPosition));
                end
                             
                self.servo_jp(jointPosition);
                tjMatrix(i, 1) = jointPosition(1);
                tjMatrix(i, 2) = jointPosition(2);
                tjMatrix(i, 3) = jointPosition(3);
                tjMatrix(i, 4) = jointPosition(4);
                tjMatrix(i, 5) = currentTime;
            end
        end % End run_trajectory()

        % Takes in desired point to go to and the total time the trajector 
        % should take, which is the same duration used in cubic_traj.
        % It then calculates the joint poses and moves the robot while
        % saving time and joint angle data.
        % It returns the saved matrix data.
        function [tjMatrix] = run_trajectory_point(self, setpoint, point2, travelTime, jointSpace)
            trajPlanner = Traj_Planner();

%             setpoint = self.setpoint_js();
            % Calculating Trajectory Coefficients
            trajCoef1 = trajPlanner.cubic_traj(0, travelTime, 0, 0,  setpoint(1), point2(1));
            trajCoef2 = trajPlanner.cubic_traj(0, travelTime, 0, 0,  setpoint(2), point2(2));
            trajCoef3 = trajPlanner.cubic_traj(0, travelTime, 0, 0,  setpoint(3), point2(3));
            trajCoef4 = trajPlanner.cubic_traj(0, travelTime, 0, 0,  setpoint(4), point2(4));
            
            % Putting all the Trajectory Coefficients into a 4x4 matrix
            trajCoef = [trajCoef1, trajCoef2, trajCoef3, trajCoef4];
            
            tic;
            currentTime = 0;
            i = 0;
            while currentTime < travelTime
                i = i + 1;
                currentTime = toc;
                
                if height(trajCoef) == 4
                    for n = 1:width(trajCoef)
                        trajectory(n) = trajCoef(1,n) + trajCoef(2,n)*currentTime + trajCoef(3,n)*currentTime^2 + trajCoef(4,n)*currentTime^3;
                    end
                elseif height(trajCoef) == 6
                    for n = 1:width(trajCoef)
                        trajectory(n) = trajCoef(1,n) + trajCoef(2,n)*currentTime + trajCoef(3,n)*currentTime^2 + trajCoef(4,n)*currentTime^3 + ...
                        trajCoef(5, n)*currentTime^4 + trajCoef(6, n)*currentTime^5;
                    end 
                end
                
                if jointSpace == true
                    jointPosition = [trajectory(1), trajectory(2), trajectory(3), trajectory(4)];
                elseif jointSpace == false
                    taskPosition = [trajectory(1), trajectory(2), trajectory(3), trajectory(4)];
                    jointPosition = rad2deg(self.ik3001(taskPosition));
                end
                             
                self.servo_jp(jointPosition);
                tjMatrix(i, 1) = jointPosition(1);
                tjMatrix(i, 2) = jointPosition(2);
                tjMatrix(i, 3) = jointPosition(3);
                tjMatrix(i, 4) = jointPosition(4);
                tjMatrix(i, 5) = currentTime;
            end
        end % End run_trajectory()

        % Takes in trajector coefficients (4x4 Matrix) with the
        % coefficients of all 4 joints and the total time the trajector 
        % should take, which is the same duration used in cubic_traj.
        % It then calculates the joint poses and moves the robot while
        % saving time and joint angle data.
        % It returns the saved matrix data.
        function [tjMatrix] = run_trajectory_plot(self, trajCoef, travelTime, jointSpace)
            tic;
            currentTime = 0;
            i = 0;
            while currentTime < travelTime
                i = i + 1;
                currentTime = toc;
                
                if height(trajCoef) == 4
                    for n = 1:width(trajCoef)
                        trajectory(n) = trajCoef(1,n) + trajCoef(2,n)*currentTime + trajCoef(3,n)*currentTime^2 + trajCoef(4,n)*currentTime^3;
                    end
                elseif height(trajCoef) == 6
                    for n = 1:width(trajCoef)
                        trajectory(n) = trajCoef(1,n) + trajCoef(2,n)*currentTime + trajCoef(3,n)*currentTime^2 + trajCoef(4,n)*currentTime^3 + ...
                        trajCoef(5, n)*currentTime^4 + trajCoef(6, n)*currentTime^5;
                    end 
                end
                
                if jointSpace == true
                    jointPosition = [trajectory(1), trajectory(2), trajectory(3), trajectory(4)];
                elseif jointSpace == false
                    taskPosition = [trajectory(1), trajectory(2), trajectory(3), trajectory(4)];
                    jointPosition = rad2deg(self.ik3001(taskPosition));
                end
                             
                self.interpolate_jp(1, jointPosition);

                jointVelocity = self.measured_js(false, true);
                jointVelocity(2) = jointVelocity(2) - 61.83; % Correcting offset for theta2
                jointVelocity(3) = jointVelocity(3) - 65.952; % Correcting offset for theta3
                jointVelocity = deg2rad(jointVelocity);
                velocityCalc = self.fdk3001(deg2rad(jointPosition), (jointVelocity)');

                % Saving time, linear velocities, and angular velocities,
                % as well as magnitude
                tjMatrix(i, 1) = currentTime;
                tjMatrix(i, 2) = velocityCalc(1);
                tjMatrix(i, 3) = velocityCalc(2);
                tjMatrix(i, 4) = velocityCalc(3);
                tjMatrix(i, 5) = velocityCalc(4);
                tjMatrix(i, 6) = velocityCalc(5);
                tjMatrix(i, 7) = velocityCalc(6);
                tjMatrix(i, 8) = sqrt(velocityCalc(1)^2 + velocityCalc(2)^2 + velocityCalc(3)^2);

                hold off;
                model = Model(); % Creating model object
                model.plot_arm(deg2rad(jointPosition), velocityCalc(1:3));
                drawnow;
            end
        end % End run_trajectory_plot()
        
        % Takes in current joint angles at time function is run and returns
        % the corresponding 6x4 Jacobian matrix
        function [Jacobian] = jacob3001(self, jointAngles)
            syms Alpha1 Theta1 Theta2 Theta3 Theta4 L0 L1 L2 L3 L4;
            dhTable = [Theta1 L1 0 Alpha1;
                       Theta2 0 L2 0;
                       Theta3 0 L3 0;
                       Theta4 0 L4 0];
            
            T01 = self.dh2mat([0 L0 0 0]);
            T12 = self.dh2mat(dhTable(1,:));
            T23 = self.dh2mat(dhTable(2,:));
            T34 = self.dh2mat(dhTable(3,:));
            T45 = self.dh2mat(dhTable(4,:));
            T05 = self.dh2fk(dhTable, T01);
            
            % Finding transformation matrices from base to respective joints
            T02 = T01 * T12;
            T03 = T01 * T12 * T23;
            T04 = T01 * T12 * T23 * T34;
                        
            % Equations for FW Position Kinematics
            Xee = T05(1,4);
            Yee = T05(2,4);
            Zee = T05(3,4);
            
            % Taking derivative wrt Theta1
            XposTheta1 = diff(Xee, Theta1);
            YposTheta1 = diff(Yee, Theta1);
            ZposTheta1 = diff(Zee, Theta1);
            
            Theta1pos = [XposTheta1; YposTheta1; ZposTheta1];
            
            % Taking derivative wrt Theta2
            XposTheta2 = diff(Xee, Theta2);
            YposTheta2 = diff(Yee, Theta2);
            ZposTheta2 = diff(Zee, Theta2);
            
            Theta2pos = [XposTheta2; YposTheta2; ZposTheta2];
            
            % Taking derivative wrt Theta3
            XposTheta3 = diff(Xee, Theta3);
            YposTheta3 = diff(Yee, Theta3);
            ZposTheta3 = diff(Zee, Theta3);
            
            Theta3pos = [XposTheta3; YposTheta3; ZposTheta3];
            
            % Taking derivative wrt Theta4
            XposTheta4 = diff(Xee, Theta4);
            YposTheta4 = diff(Yee, Theta4);
            ZposTheta4 = diff(Zee, Theta4);
            
            Theta4pos = [XposTheta4; YposTheta4; ZposTheta4];
            
            

            % Finding Jos
            Jo1 = [T01(1,3); T01(2,3); T01(3,3)];
            Jo2 = [T02(1,3); T02(2,3); T02(3,3)];
            Jo3 = [T03(1,3); T03(2,3); T03(3,3)];
            Jo4 = [T04(1,3); T04(2,3); T04(3,3)];
             
            Jacobian(1:3,1:1) = Theta1pos;
            Jacobian(1:3,2:2) = Theta2pos;
            Jacobian(1:3,3:3) = Theta3pos;
            Jacobian(1:3,4:4) = Theta4pos;
            Jacobian(4:6,1:1) = Jo1;
            Jacobian(4:6,2:2) = Jo2;
            Jacobian(4:6,3:3) = Jo3;
            Jacobian(4:6,4:4) = Jo4;

            % Substituting Symbolic Alpha Value with Real value
            Jacobian = subs(Jacobian, Alpha1, -pi/2);

            % Substituting Symbolic Joint Angle Values with Real Values
            Jacobian = subs(Jacobian, Theta1, jointAngles(1));
            Jacobian = subs(Jacobian, Theta2, jointAngles(2) + atan(24/128) - pi/2);
            Jacobian = subs(Jacobian, Theta3, jointAngles(3) + pi/2 - atan(24/128));
            Jacobian = subs(Jacobian, Theta4, jointAngles(4));

            % Substituting Symbolic Link Values with Real values
            Jacobian = subs(Jacobian, L0, 36.076);
            Jacobian = subs(Jacobian, L1, 96.326-36.076);
            Jacobian = subs(Jacobian, L2, sqrt(128^2 + 24^2));
            Jacobian = subs(Jacobian, L3, 124);
            Jacobian = subs(Jacobian, L4, 133.4);

            Jacobian = double(Jacobian);
        end % End jacob3001()

        % Calculates the forward velocity kinematics that can be used in
        % real-time while the robot is running.
        % Takes in all of the current joint angles and the joint velocities
        % and returns the 6x1 vector includig task-space linear velocities
        % and angular velocities
        function [eeVelocities] = fdk3001(self, jointAngles, jointVelocities)

            jacob = self.jacob3001(jointAngles);
            eeVelocities = jacob * jointVelocities;

            disp(jacob);
            disp(eeVelocities);

        end % End fdk3001()

        % Takes in trajector coefficients (4x4 Matrix) with the
        % coefficients of all 4 joints and the total time the trajector 
        % should take, which is the same duration used in cubic_traj.
        % It then calculates the joint poses and moves the robot while
        % saving time and joint angle data.
        % It returns the saved matrix data.
        function [tjMatrix] = run_trajectory_eStop(self, trajCoef, travelTime, jointSpace)
            tic;
            currentTime = 0;
            i = 0;
            while currentTime < travelTime
                i = i + 1;
                currentTime = toc;
                
                if height(trajCoef) == 4
                    for n = 1:width(trajCoef)
                        trajectory(n) = trajCoef(1,n) + trajCoef(2,n)*currentTime + trajCoef(3,n)*currentTime^2 + trajCoef(4,n)*currentTime^3;
                    end
                elseif height(trajCoef) == 6
                    for n = 1:width(trajCoef)
                        trajectory(n) = trajCoef(1,n) + trajCoef(2,n)*currentTime + trajCoef(3,n)*currentTime^2 + trajCoef(4,n)*currentTime^3 + ...
                        trajCoef(5, n)*currentTime^4 + trajCoef(6, n)*currentTime^5;
                    end 
                end
                
                if jointSpace == true
                    jointPosition = [trajectory(1), trajectory(2), trajectory(3), trajectory(4)];
                elseif jointSpace == false
                    taskPosition = [trajectory(1), trajectory(2), trajectory(3), trajectory(4)];
                    jointPosition = rad2deg(self.ik3001(taskPosition));
                end
                             
                self.interpolate_jp(1, jointPosition);
                Jacob = self.jacob3001(deg2rad(jointPosition));
                detJacob = det(Jacob(1:3, 1:3));

                % Stops robot if it is close to singularity
                if round(detJacob/10000000, 1) == 0
                    error("Robot is reaching singularity");
                end
                
                Positions = self.fk3001(deg2rad(jointPosition'));
                tjMatrix(i, 1) = currentTime;
                tjMatrix(i, 2) = Positions(1, 4);
                tjMatrix(i, 3) = Positions(2, 4);
                tjMatrix(i, 4) = Positions(3, 4);
                tjMatrix(i, 5) = detJacob;                       

                hold off;
                model = Model(); % Creating model object
                model.plot_arm(deg2rad(jointPosition), [0 0 0 0]', round(detJacob/10000000, 1));
                drawnow;
            end
        end % End run_trajectory()

    end % end methods
end % end class 
