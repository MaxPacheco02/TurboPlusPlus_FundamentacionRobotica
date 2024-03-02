close all
clear all

l1 = 10;
l2 = 10;
l3 = 5;
t1 = 0;
t2 = -pi/4;
t3 = -pi/4;

xd = 0;
yd = 0;
zd = -0.1;

theta_dot = [0; 0; -0.1];
dt = 1;

for i = 0:dt:10

J = [[l3*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)) - l1*sin(t1) - l2*cos(t2)*sin(t1), - l3*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)) - l2*cos(t1)*sin(t2), -l3*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2))];
[l1*cos(t1) - l3*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) + l2*cos(t1)*cos(t2), - l3*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)) - l2*sin(t1)*sin(t2), -l3*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2))];
[0, -l3*(cos(t2)*cos(t3) - sin(t2)*sin(t3)) - l2*cos(t2), -l3*(cos(t2)*cos(t3) - sin(t2)*sin(t3))]];

theta_dot = inv(J)' * theta_dot;

t1 = t1 + theta_dot(1)*dt;
t2 = t2 + theta_dot(2)*dt;
t3 = t3 + theta_dot(3)*dt;

x = l1*cos(t1) - l3*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) + l2*cos(t1)*cos(t2);
y = l1*sin(t1) - l3*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)) + l2*cos(t2)*sin(t1);
z = - l3*(cos(t2)*sin(t3) + cos(t3)*sin(t2)) - l2*sin(t2);
plot3(x,y,z,'k*')
hold on
end