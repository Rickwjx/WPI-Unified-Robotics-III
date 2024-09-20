classdef Traj_Planner < Robot

    properties 
        robot; % Stores a robot instance
    end
    
    methods
        
        function self = Traj_Planner()
            self.robot = Robot();
        end

        % Takes in time, postion, and velocity values at two points and
        % returns cubic polynomial trajectory coefficients
        function [coefficients] = cubic_traj(self, t0, tf, v0, vf, q0, qf)
            
            M = [1, t0, t0^2, t0^3; 
                0, 1, 2*t0, 3*t0^2;
                1, tf, tf^2, tf^3;
                0, 1, 2*tf, 3*tf^2]
            Minv = inv(M); %Inversing M

            B = [q0; v0; qf; vf];

            coefficients = Minv * B;
    
        end % End cubic_traj

        function [coefficients] = quintic_traj(self, t0, tf, v0, vf, q0, qf, alpha0, alphaf)
            
            M = [1, t0, t0^2, t0^3, t0^4, t0^5; 
                 0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
                 0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
                 1, tf, tf^2, tf^3, tf^4, tf^5;
                 0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
                 0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];

            Minv = inv(M); %Inversing M

            B = [q0; v0; alpha0; qf; vf; alphaf];

            coefficients = Minv * B;
    
        end % End quintic_traj

    end % End Methods
end % End Class