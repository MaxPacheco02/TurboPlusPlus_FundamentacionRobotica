figure(1);
clear all
clc

% Create a hexagonal prism
radius = 1;      % Radius of the base of the hexagonal prism
height = 1;      % Height of the hexagonal prism
num_sides = 6;   % Number of sides of the hexagon

% Define the hexagonal prism
theta = linspace(0,2*pi,num_sides+1); % angles for the hexagon
z = [zeros(size(theta)); height*ones(size(theta))]; % z-coordinates
x = radius*cos(theta); % x-coordinates
y = radius*sin(theta); % y-coordinates

% Initialize the animated line
h = animatedline('Color','r','LineWidth',2);

% Plot the hexagonal prism
x_list = [0];
y_list = [0];
z_list = [0];
for i = 1:size(x, 2)-1
    for j = 1:size(z, 1)
        n = 5;
        x_l = linspace(x_list(size(x_list,2)), x(i), n);
        y_l = linspace(y_list(size(y_list,2)), y(i), n);
        z_l = linspace(z_list(size(z_list,2)), z(j, i), n);
        x_list = [x_list,x_l(2:end)];
        y_list = [y_list,y_l(2:end)];
        z_list = [z_list,z_l(2:end)];
    end
end

% Add the top and bottom faces of the hexagonal prism
for r = linspace(0, radius, 1) % draw concentric hexagons
    x = r*cos(theta); % x-coordinates for the top and bottom faces
    y = r*sin(theta); % y-coordinates for the top and bottom faces
    z = zeros(size(theta)); % z-coordinates for the bottom face
    x_list = [x_list, x, x];
    y_list = [y_list, y, y];
    z_list = [z_list, z, z+height];
end

for i = 1:size(x_list,2)
    addpoints(h, x_list(i), y_list(i), z_list(i));
    drawnow limitrate;
end

csv_file_path = 'hexagonal_prism.csv'; % Specify the output CSV file path
writematrix([x_list;y_list;z_list]', csv_file_path);
