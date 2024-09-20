
%% Setup robot
travelTime = 2; % Defines the travel time
robot = Robot(); % Creates robot object
% robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

% Symbolic Part 4
syms Alpha1 Theta1 Theta2 Theta3 Theta4 L0 L1 L2 L3 L4
dhTable = [Theta1 L1 0 Alpha1;
           Theta2 0 L2 0;
           Theta3 0 L3 0;
           Theta4 0 L4 0];

T01 = robot.dh2mat([0 L0 0 0]);
T12 = robot.dh2mat(dhTable(1,:));
T23 = robot.dh2mat(dhTable(2,:));
T34 = robot.dh2mat(dhTable(3,:));
T45 = robot.dh2mat(dhTable(4,:));
T05 = robot.dh2fk(dhTable, T01);

% Finding transformation matricies from base to respeictive joints
T02 = T01 * T12;
T03 = T01 * T12 * T23;
T04 = T01 * T12 * T23 + T34;

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
z1 = [T01(1,3); T01(2,3); T01(3,3)];
z2 = [T02(1,3); T02(2,3); T02(3,3)];
z3 = [T03(1,3); T03(2,3); T03(3,3)];
z4 = [T04(1,3); T04(2,3); T04(3,3)];

Jo1 = z1;
Jo2 = z2;
Jo3 = z3;
Jo4 = z4;

Jacobian(1:3,1:1) = Theta1pos;
Jacobian(1:3,2:2) = Theta2pos;
Jacobian(1:3,3:3) = Theta3pos;
Jacobian(1:3,4:4) = Theta4pos;
Jacobian(4:6,1:1) = Jo1;
Jacobian(4:6,2:2) = Jo2;
Jacobian(4:6,3:3) = Jo3;
Jacobian(4:6,4:4) = Jo4;