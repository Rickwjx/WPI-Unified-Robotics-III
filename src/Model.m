classdef Model < Robot


    properties 
        robot; % Stores a robot instance
    end

    methods
        function self = Model()
            self.robot = Robot();
        end

        function plot_arm(self, q, qDot, det) 
            
            L1 = 93.326 - 36.076;
            L2 = sqrt(128^2 + 24^2);
            L3 = 124; 
            L4 = 133.4;
 
            alpha1 = -pi/2;
            theta2 = atan(24 / 128) - (pi/2);
            theta3 = (pi/2) - atan(24 / 128);
            theta4 = 0;

            TMatrix = self.dh2mat([0 36.076 0 0]);

            modifiedMatrix = [q(1),          L1,  0,    alpha1;
                              theta2+q(2),   0,   L2,   0;
                              theta3+q(3),   0,   L3,   0;
                              theta4+q(4),   0,   L4,   0];

            jointPositions = zeros(4, 3);
            jointRotations = zeros(4, 9);

            for n = 1:4
                TMatrix = self.dh2fk(modifiedMatrix(n, :), TMatrix);

                jointPositions(n, 1) = TMatrix(1, 4);
                jointPositions(n, 2) = TMatrix(2, 4);
                jointPositions(n, 3) = TMatrix(3, 4);

                jointRotations(n, 1) = TMatrix(1,1);
                jointRotations(n, 2) = TMatrix(2,1);
                jointRotations(n, 3) = TMatrix(3,1);
                jointRotations(n, 4) = TMatrix(1,2);
                jointRotations(n, 5) = TMatrix(2,2);
                jointRotations(n, 6) = TMatrix(3,2);
                jointRotations(n, 7) = TMatrix(1,3);
                jointRotations(n, 8) = TMatrix(2,3);
                jointRotations(n, 9) = TMatrix(3,3);
            end

            J0x = 0;
            J0y = 0;
            J0z = 0;

            J1x = jointPositions(1,1);
            J1y = jointPositions(1,2);
            J1z = jointPositions(1,3);

            J2x = jointPositions(2,1);
            J2y = jointPositions(2,2);
            J2z = jointPositions(2,3);

            J3x = jointPositions(3,1);
            J3y = jointPositions(3,2);
            J3z = jointPositions(3,3);

            J4x = jointPositions(4,1);
            J4y = jointPositions(4,2);
            J4z = jointPositions(4,3);


            J1x1x0 = jointRotations(1,1);
            J1x1y0 = jointRotations(1,2);
            J1x1z0 = jointRotations(1,3);
            J1y1x0 = jointRotations(1,4);
            J1y1y0 = jointRotations(1,5);
            J1y1z0 = jointRotations(1,6);    
            J1z1x0 = jointRotations(1,7);
            J1z1y0 = jointRotations(1,8);
            J1z1z0 = jointRotations(1,9);

            J2x1x0 = jointRotations(2,1);
            J2x1y0 = jointRotations(2,2);
            J2x1z0 = jointRotations(2,3);
            J2y1x0 = jointRotations(2,4);
            J2y1y0 = jointRotations(2,5);
            J2y1z0 = jointRotations(2,6);    
            J2z1x0 = jointRotations(2,7);
            J2z1y0 = jointRotations(2,8);
            J2z1z0 = jointRotations(2,9);   
 
            J3x1x0 = jointRotations(3,1);
            J3x1y0 = jointRotations(3,2);
            J3x1z0 = jointRotations(3,3);
            J3y1x0 = jointRotations(3,4);
            J3y1y0 = jointRotations(3,5);
            J3y1z0 = jointRotations(3,6);    
            J3z1x0 = jointRotations(3,7);
            J3z1y0 = jointRotations(3,8);
            J3z1z0 = jointRotations(3,9);


            J4x1x0 = jointRotations(4,1);
            J4x1y0 = jointRotations(4,2);
            J4x1z0 = jointRotations(4,3);
            J4y1x0 = jointRotations(4,4);
            J4y1y0 = jointRotations(4,5);
            J4y1z0 = jointRotations(4,6);    
            J4z1x0 = jointRotations(4,7);
            J4z1y0 = jointRotations(4,8);
            J4z1z0 = jointRotations(4,9);

            plot3([J0x J1x], [J0y J1y], [J0z J1z], ...
                  [J1x J2x], [J1y J2y], [J1z J2z], ...
                  [J2x J3x], [J2y J3y], [J2z J3z], ...
                  [J3x J4x], [J3y J4y], [J3z J4z], ...
                  'LineWidth', 2);
           
            grid on;
            hold on;

            title('Live Plot of Arm');
            xlabel('X Position (mm)');
            ylabel('Y Position (mm)');
            zlabel('Z Position (mm)');

            quiver3(0, 0, 0, 1, 0, 0);
            quiver3(0, 0, 0, 0, 1, 0);
            quiver3(0, 0, 0, 0, 0, 1);
 
            quiver3(J1x, J1y, J1z, J1x1x0, J1x1y0, J1x1z0, 50, 'LineWidth',1);
            quiver3(J1x, J1y, J1z, J1y1x0, J1y1y0, J1y1z0, 50, 'LineWidth',1);
            quiver3(J1x, J1y, J1z, J1z1x0, J1z1y0, J1z1z0, 50, 'LineWidth',1);

            quiver3(J2x, J2y, J2z, J2x1x0, J2x1y0, J2x1z0, 50, 'LineWidth',1);
            quiver3(J2x, J2y, J2z, J2y1x0, J2y1y0, J2y1z0, 50, 'LineWidth',1);
            quiver3(J2x, J2y, J2z, J2z1x0, J2z1y0, J2z1z0, 50, 'LineWidth',1);

            quiver3(J3x, J3y, J3z, J3x1x0, J3x1y0, J3x1z0, 50, 'LineWidth',1);
            quiver3(J3x, J3y, J3z, J3y1x0, J3y1y0, J3y1z0, 50, 'LineWidth',1);
            quiver3(J3x, J3y, J3z, J3z1x0, J3z1y0, J3z1z0, 50, 'LineWidth',1);

            quiver3(J4x, J4y, J4z, J4x1x0, J4x1y0, J4x1z0, 50, 'LineWidth',1);
            quiver3(J4x, J4y, J4z, J4y1x0, J4y1y0, J4y1z0, 50, 'LineWidth',1);
            quiver3(J4x, J4y, J4z, J4z1x0, J4z1y0, J4z1z0, 50, 'LineWidth',1);

            veloMagnitude = sqrt(qDot(1)^2 + qDot(2)^2 + qDot(3)^2);
            disp([qDot(1), qDot(2), qDot(3)]);
            quiver3(J4x, J4y, J4z, qDot(1), qDot(2), qDot(3), 0.01*veloMagnitude, 'LineWidth', 1);

            if det == 0
                text(-400, 400, 0, sprintf("Robot is reaching singularity"),...
                        'Color', 'r', 'FontSize', 14, 'FontWeight', 'bold');
            end

            axis equal;
            axis([-1*(L1+L2+L3+L4) L1+L2+L3+L4 -1*(L1+L2+L3+L4) L1+L2+L3+L4 -1 L1+L2+L3+L4])
        end
    end
end