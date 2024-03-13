close all
clear all

syms l1 l2 l3 t1 t2 t3

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
m2 = [rot2,[l1;0;0];0,0,0,1];
m3 = [rot3,[l2;0;0];0,0,0,1];
m4 = [rotI,[l3;0;0];0,0,0,1];
mR = m1*m2*m3*m4;

x = l1*cos(t1) - l3*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) + l2*cos(t1)*cos(t2);
y = l1*sin(t1) - l3*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)) + l2*cos(t2)*sin(t1);
z = l3*(cos(t2)*sin(t3) + cos(t3)*sin(t2)) + l2*sin(t2);

J = [
    diff(x,t1), diff(x,t2), diff(x,t3);
    diff(y,t1), diff(y,t2), diff(y,t3);
    diff(z,t1), diff(z,t2), diff(z,t3);    
    ];