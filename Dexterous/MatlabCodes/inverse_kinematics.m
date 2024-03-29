close all
clear all
clc

l1 = 10; % Length of arms
l2 = 10;
l3 = 5;
t1 = 0; % Angle of joints
t2 = 1.1;
t3 = 2.3;

vel_dot = [0; 0.01; 0.01]; % Vector velocity
dt = 1;

q_list = [];

for i = 0:dt:500

q_list = [q_list; [t1, t2, t3]];

% graph the whole finger huh
p = [0,0,0];
rot1 = [cos(t1), -sin(t1), 0;
    sin(t1), cos(t1), 0;
    0, 0, 1;
];
rot2 = [cos(t2), 0, -sin(t2);
    0, 1, 0;
    sin(t2), 0, cos(t2);
];
rot3 = [cos(t3), 0, -sin(t3);
    0, 1, 0;
    sin(t3), 0, cos(t3);
];

rotI = eye(3);

m1 = [rot1,[0;0;0];0,0,0,1];
m2 = [rotI,[l1;0;0];0,0,0,1];
mR = m1*m2;
p = [p; mR(1,4),mR(2,4),mR(3,4)];
plot3(mR(1,4),mR(2,4), mR(3,4),'ko');
hold on

m2 = [rot2,[l1;0;0];0,0,0,1];
m3 = [rotI,[l2;0;0];0,0,0,1];
mR = m1*m2*m3;
p = [p; mR(1,4),mR(2,4),mR(3,4)];
plot3(mR(1,4),mR(2,4), mR(3,4),'ko');
hold on

m3 = [rot3,[l2;0;0];0,0,0,1];
m4 = [rotI,[l3;0;0];0,0,0,1];
mR = m1*m2*m3*m4;
p = [p; mR(1,4),mR(2,4),mR(3,4)];
plot3(mR(1,4),mR(2,4), mR(3,4),'ko');
hold on

plot3(p(:,1),p(:,2),p(:,3),'-');

x = l1*cos(t1) - l3*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) + l2*cos(t1)*cos(t2);
y = l1*sin(t1) - l3*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)) + l2*cos(t2)*sin(t1);
z = l3*(cos(t2)*sin(t3) + cos(t3)*sin(t2)) + l2*sin(t2);
[x,y,z];
plot3(x,y,z,'r*');
hold on

% % Jacobian (Calculated from derivative of forward kinematics)
J = [[l3*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)) - l1*sin(t1) - l2*cos(t2)*sin(t1), - l3*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)) - l2*cos(t1)*sin(t2), -l3*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2))];
[l1*cos(t1) - l3*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) + l2*cos(t1)*cos(t2), - l3*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)) - l2*sin(t1)*sin(t2), -l3*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2))];
[0, l3*(cos(t2)*cos(t3) - sin(t2)*sin(t3)) + l2*cos(t2), l3*(cos(t2)*cos(t3) - sin(t2)*sin(t3))]];

theta_dot = J\vel_dot; % Calculating angular velocities

% Calculating new angles
% based on previous angles
t1 = t1 + theta_dot(1)*dt; 
t2 = t2 + theta_dot(2)*dt;
t3 = t3 + theta_dot(3)*dt;

% if t1 < -1
%     t1 = -1;
% elseif t1 > 1
%     t1 = 1;
% end
% if t2 < 0
%     t2 = 0;
% elseif t2 > 2
%     t2 = 2;
% end
% if t3 < 0
%     t3 = 0;
% elseif t3 > 2
%     t3 = 2;
% end
end
