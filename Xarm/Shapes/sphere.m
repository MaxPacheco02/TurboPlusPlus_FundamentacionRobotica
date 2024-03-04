figure(1);
clear all
clc

% Create a sphere
radius = 1;      % Radius of the sphere

% Define the sphere
[theta, phi] = meshgrid(linspace(0,2*pi,20), linspace(-pi/2,pi/2,20)); % meshgrid for the sphere
x = radius*cos(theta).*cos(phi); % x-coordinates
y = radius*sin(theta).*cos(phi); % y-coordinates
z = radius*sin(phi); % z-coordinates

% Initialize the animated line
h = animatedline('Color','b','LineWidth',2);

% Plot the sphere
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
csv_file_path = 'sphere.csv'; % Specify the output CSV file path
writematrix([x_list;y_list;z_list]', csv_file_path);
