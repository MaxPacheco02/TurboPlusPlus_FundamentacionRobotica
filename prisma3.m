
% BOX VERTICES
Lengthb = 10;
Heightb = 3;
Widthb = 5;

la = Lengthb/2;
ha = Heightb/2;
wa = Widthb/2;

% CENTERED AROUND (0,0,0)
A = [wa;la;ha];
B = [wa;la;-ha];
C = [wa;-la;-ha];
D = [wa;-la;ha];
E = [-wa;-la;ha];
F = [-wa;-la;-ha];
G = [-wa;la;-ha];
H = [-wa;la;ha];

%POSITION OF CM AND CONTACT POINTS IN SPACE
PCM = [0;0;0];
P1 = [0;-la/2;ha];
P2 = [0;la/2;ha];
P3 = [0;0;-ha];
points = setRefPos(P1,P2,P3,la,ha,wa);

if points == 0
    % DEFAULT VALUES (IDEAL CONTACT POINTS)
    P1 = [0;-la/2;ha];
    P2 = [0;la/2;ha];
    P3 = [0;0;-ha];
end

%FRAME VECTORS
PCMa = [[1; 0; 0] , [0; 1; 0] , [0; 0; 1]];
P1a = PCMa;
P2a = PCMa;
P3a = PCMa;

%TRANSLATION VECTOR
dis = [2;2;2];

%RADIAN ROTATION
xang = -320;
yang = -pi/4;
zang = 45;

%ROTATION MATRICES
Rx = [1, 0, 0; 0, cos(xang), -sin(xang); 0, sin(xang), cos(xang)];
Ry = [cos(yang), 0, sin(yang); 0, 1, 0; -sin(yang), 0, cos(yang)];
Rz = [cos(zang), -sin(zang), 0; sin(zang), cos(zang), 0; 0, 0, 1];
R = Rx * Ry * Rz;

%PERFORM ROTATION 
PCMar = R * PCMa;
P1ar = R * P1a;
P2ar = R * P2a;
P3ar = R * P3a;

%PERFORM TRANSLATION
PCMd = PCM + dis;
%Rotate and translate the entire position vector
P1d = (R * P1) + dis;
P2d = (R * P2) + dis;
P3d = (R * P3) + dis;

%--------------------------------------------------
%Forward Kiematics
H0 = [1,0,0,dis(1);0,1,0,dis(2);0,0,1,dis(3);0,0,0,1];
H1 = [Ry, [0;0;0]; 0, 0, 0, 1];

Ht = H0 * H1;

PCMa = [PCMa; [1,1,1]];

posN = Ht * PCMa;
%--------------------------------------------------

% BOX INITIAL AND FINAL POSITION
BOX = [A,B,C,D,E,F,G,H];
BOXq = (R*BOX) + dis;

l = 15;
% ORIGINAL POSITION IN SPACE
figure
axis([-l l -l l -l l]);
hold all
drawAx(PCM,PCMa);
drawAx(P1,P1a);
drawAx(P2,P2a);
drawAx(P3,P3a);
drawVe(PCM,[P1,P2,P3]);
xlabel('X-axisx');
ylabel('Y-axis');
zlabel('Z-axis');
drawBox(BOX(:,1),BOX(:,2),BOX(:,3),BOX(:,4),BOX(:,5),BOX(:,6),BOX(:,7),BOX(:,8));
view(20,20)
set(gca, 'LineWidth',2, 'XGrid','on', 'GridLineStyle','--');

% NEW POSITION IN SPACE
figure 
axis([-l l -l l -l l]);
hold all
drawAx(PCMd, PCMar);
drawAx(P1d,P1ar);
drawAx(P2d, P2ar);
drawAx(P3d, P3ar);
drawVe(PCMd, [P1d, P2d, P3d] - dis);
xlabel('X-axisx');
ylabel('Y-axis');
zlabel('Z-axis');
drawBox(BOXq(:,1),BOXq(:,2),BOXq(:,3),BOXq(:,4),BOXq(:,5),BOXq(:,6),BOXq(:,7),BOXq(:,8));
view(20,20)
set(gca, 'LineWidth',2, 'XGrid','on', 'GridLineStyle','--');


% DRAW UNIT FRAMES
function drawAx(POS, MAG)
    quiver3(POS(1),POS(2),POS(3),MAG(1,1),MAG(2,1),MAG(3,1),'b','LineWidth',2); %unit x
    quiver3(POS(1),POS(2),POS(3),MAG(1,2),MAG(2,2),MAG(3,2),'r','LineWidth',2); %unit y
    quiver3(POS(1),POS(2),POS(3),MAG(1,3),MAG(2,3),MAG(3,3),'g','LineWidth',2); %unit z
end

% DRAW UNIT FRAMES POSITION VECTORS 
function drawVe(POS, MAG)
    quiver3(POS(1),POS(2),POS(3),MAG(1,1),MAG(2,1),MAG(3,1),'k','LineWidth',2); 
    quiver3(POS(1),POS(2),POS(3),MAG(1,2),MAG(2,2),MAG(3,2),'k','LineWidth',2); 
    quiver3(POS(1),POS(2),POS(3),MAG(1,3),MAG(2,3),MAG(3,3),'k','LineWidth',2); 
end

% DRAW BOX (CHANGE)
function drawBox(A,B,C,D,E,F,G,H)
    Box = [A,H,E,D,A,B,C,D,C,F,E,F,G,H,G,B]; 
    for i = 1:15
        line([Box(1,i),Box(1,i + 1)], [Box(2,i),Box(2,i + 1)], [Box(3,i),Box(3,i + 1)], 'Color', 'red', 'LineWidth', 2);
    end
end

%CHECH IF CONTACT POINTS ARE WITHIN BOX DIMENSIONS
function points = setRefPos(p1,p2,p3,l,h,w)
    if (p1(1) > w || p1(1) < -w) || (p2(1) > w || p2(1) < -w) || (p3(1) > w || p3(1) < -w)
        points = 0;
    elseif (p1(2) > l || p1(2) < -l) || (p2(2) > l || p2(2) < -l) || (p3(2) > l || p3(2) < -l)
        points = 0;
    elseif (p1(2) > h || p1(2) < -h) || (p2(2) > h || p2(2) < -h) || (p3(2) > h || p3(2) < -h)
        points = 0;
    else
        points = 1;
    end
end