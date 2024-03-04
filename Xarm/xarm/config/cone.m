figure(1);
clear all
clc

% Create a cone
radius = 1;      % Radius of the base of the cone
height = 1;      % Height of the cone

% Define the cone
[theta, z] = meshgrid(linspace(0,2*pi,20), linspace(0,height,20)); % meshgrid for the cone
x = radius*(1-z/height).*cos(theta); % x-coordinates
y = radius*(1-z/height).*sin(theta); % y-coordinates

% Initialize the animated line
h = animatedline('Color','g','LineWidth',2);

% Plot the cone
x_list = [0];
y_list = [0];
z_list = [0];
for i = 1:size(x, 1)
    for j = 1:size(y, 2)
        n = 2;
        x_l = linspace(x_list(size(x_list,2)), x(i, j), n);
        y_l = linspace(y_list(size(y_list,2)), y(i, j), n);
        z_l = linspace(z_list(size(z_list,2)), z(i, j), n);
        x_list = [x_list,x_l(2:end)];
        y_list = [y_list,y_l(2:end)];
        z_list = [z_list,z_l(2:end)];
    end
end

for i = 1:size(x_list,2)
    addpoints(h, x_list(i), y_list(i), z_list(i));
    drawnow limitrate;
    x_list(i) = x_list(i) + 0.2;
    y_list(i) = y_list(i) + 0.2;
    z_list(i) = z_list(i) + 0.2;
end

csv_file_path = 'cone.csv'; % Specify the output CSV file path
writematrix([x_list;y_list;z_list]', csv_file_path);
