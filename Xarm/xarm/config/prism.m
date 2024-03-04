figure(1);
clear all
clc

% Create a rectangular prism
width = 1;      % Width of the prism
height = 0.5;     % Height of the prism
depth = 1;      % Depth of the prism

% Define the vertices of the rectangular prism
vertices = [
    0, 0, 0;          % Vertex 1
    width, 0, 0;      % Vertex 2
    width, height, 0; % Vertex 3
    0, height, 0;     % Vertex 4
    0, 0, depth;      % Vertex 5
    width, 0, depth;  % Vertex 6
    width, height, depth; % Vertex 7
    0, height, depth  % Vertex 8
];

% Define the edges connecting the vertices
edges = [
    1, 2; 2, 3; 3, 4; 4, 1; % Bottom face
    1, 5; 5, 6; 6,2; 2,3; 3,7;
    7,8; 8,4; 4,1; 1,5;
    5, 6; 6, 7; 7, 8; 8, 5;
];

% Initialize the animated line
h = animatedline('Color','g','LineWidth',2);

% Plot the rectangular prism
x_list = [0];
y_list = [0];
z_list = [0];
for i = 1:size(edges, 1)
    x = vertices(edges(i, :), 1);
    y = vertices(edges(i, :), 2);
    z = vertices(edges(i, :), 3);
    n = 10;
    x_l = linspace(x_list(size(x_list,2)), x(2), n);
    y_l = linspace(y_list(size(y_list,2)), y(2), n);
    z_l = linspace(z_list(size(z_list,2)), z(2), n);
    x_list = [x_list,x_l(2:end)];
    y_list = [y_list,y_l(2:end)];
    z_list = [z_list,z_l(2:end)];
end

for i = 1:size(x_list,2)
    addpoints(h, x_list(i), y_list(i), z_list(i));
    drawnow limitrate;
    x_list(i) = x_list(i) + 0.2;
    y_list(i) = y_list(i) + 0.2;
    z_list(i) = z_list(i) + 0.2;
end

csv_file_path = 'prism.csv'; % Specify the output CSV file path
writematrix([x_list;y_list;z_list]', csv_file_path);