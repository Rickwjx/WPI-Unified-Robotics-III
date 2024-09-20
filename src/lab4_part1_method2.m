
%% Setup robot
travelTime = 2; % Defines the travel time
robot = Robot(); % Creates robot object
%robot.writeTime(travelTime); % Write travel time
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

pe = [T05(1,4); T05(2,4); T05(3,4)];

%Finding transformation matricies from base to respeictive joints
T02 = T01 * T12;
T03 = T01 * T12 * T23;
T04 = T01 * T12 * T23 + T34;

%Finding Jp1
p01 = [T01(1,4); T01(2,4); T01(3,4)];
z1 = [T01(1,3); T01(2,3); T01(3,3)];
Jp1 = cross(z1, (pe - p01));

%Finding Jp2
p02 = [T02(1,4); T02(2,4); T02(3,4)];
z2 = [T02(1,3); T02(2,3); T02(3,3)];
Jp2 = cross(z2, (pe - p02));

%Finding Jp3
p03 = [T03(1,4); T03(2,4); T03(3,4)];
z3 = [T03(1,3); T03(2,3); T03(3,3)];
Jp3 = cross(z3, (pe - p03));

%Finding Jp4
p04 = [T04(1,4); T04(2,4); T04(3,4)];
z4 = [T04(1,3); T04(2,3); T04(3,3)];
Jp4 = cross(z4, (pe - p04));


%Finding Jos
Jo1 = z1;
Jo2 = z2;
Jo3 = z3;
Jo4 = z4;

%Initializes empty jacobian matrix

Jacobian2(1:3,1:1) = Jp1;
Jacobian2(1:3,2:2) = Jp2;
Jacobian2(1:3,3:3) = Jp3;
Jacobian2(1:3,4:4) = Jp4;
Jacobian2(4:6,1:1) = Jo1;
Jacobian2(4:6,2:2) = Jo2;
Jacobian2(4:6,3:3) = Jo3;
Jacobian2(4:6,4:4) = Jo4;