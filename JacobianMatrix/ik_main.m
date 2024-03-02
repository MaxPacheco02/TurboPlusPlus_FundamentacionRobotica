% Example usage:
goal_pose = [10, 5, 5];  % Goal position in x, y, z
l1 = 10;  % Length of link 1
l2 = 10;  % Length of link 2
l3 = 5;   % Length of link 3

[theta1, theta2, theta3] = inverse_kinematics(goal_pose, l1, l2, l3);

fprintf('Joint angles: Theta1 = %.2f, Theta2 = %.2f, Theta3 = %.2f\n', theta1, theta2, theta3);