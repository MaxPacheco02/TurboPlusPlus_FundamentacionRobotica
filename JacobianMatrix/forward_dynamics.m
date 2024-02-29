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
for i = 0:0.1:2*pi/3
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
    plot3(mR(1,4),mR(2,4), mR(3,4),'ro');
    hold on

    m2 = [rot2,[l1;0;0];0,0,0,1];
    m3 = [rotI,[l2;0;0];0,0,0,1];
    mR = m1*m2*m3;
    plot3(mR(1,4),mR(2,4), mR(3,4),'go');
    hold on

    m3 = [rot3,[l2;0;0];0,0,0,1];
    m4 = [rotI,[l3;0;0];0,0,0,1];
    mR = m1*m2*m3*m4;
    plot3(mR(1,4),mR(2,4), mR(3,4),'bo');
    hold on

end


% m2(1,3) = l1;
% m3(1,3) = l2;
% 
% [x1,y1] = (m1*m2)*m3;


