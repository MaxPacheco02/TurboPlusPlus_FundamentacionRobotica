close all
clear all

l1 = 10;
l2 = 10;
l3 = 5;
t1 = 0;
t2 = 0;
t3 = 0;

figure()
axis([0 30 0 30 0 30]);
hold on
grid on

i = 0.3;
p = [0,0,0];
t1 = i;
t2 = -pi/4;
t3 = -pi/4;
rot1 = [cos(t1), -sin(t1), 0;
    sin(t1), cos(t1), 0;
    0, 0, 1;
];
rot2 = [cos(t2), 0, sin(t2);
    0, 1, 0;
    -sin(t2), 0, cos(t2);
];
rot3 = [cos(t3), 0, sin(t3);
    0, 1, 0;
    -sin(t3), 0, cos(t3);
];

rotI = eye(3);

m1 = [rot1,[0;0;0];0,0,0,1];
m2 = [rotI,[l1;0;0];0,0,0,1];
mR = m1*m2;
p = [p; mR(1,4),mR(2,4),mR(3,4)];
plot3(mR(1,4),mR(2,4), mR(3,4),'ro');
hold on

m2 = [rot2,[l1;0;0];0,0,0,1];
m3 = [rotI,[l2;0;0];0,0,0,1];
mR = m1*m2*m3;
p = [p; mR(1,4),mR(2,4),mR(3,4)];
plot3(mR(1,4),mR(2,4), mR(3,4),'go');
hold on

m3 = [rot3,[l2;0;0];0,0,0,1];
m4 = [rotI,[l3;0;0];0,0,0,1];
mR = m1*m2*m3*m4;
p = [p; mR(1,4),mR(2,4),mR(3,4)];
plot3(mR(1,4),mR(2,4), mR(3,4),'bo');
hold on

plot3(p(:,1),p(:,2),p(:,3),'-');
x = l1*cos(t1) - l3*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) + l2*cos(t1)*cos(t2);
y = l1*sin(t1) - l3*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)) + l2*cos(t2)*sin(t1);
z = - l3*(cos(t2)*sin(t3) + cos(t3)*sin(t2)) - l2*sin(t2);
plot3(x,y,z,'k*');
