close all;
clc;
clear;

% Delta robot constants
global R r L l
R=120;
r=30;
L=90;
l=250;

theta = linspace(0, 2 * pi, 100);
% Define first circle points in the XY plane
x1 = R * cos(theta); % X-axis points for the first circle
y1 = R * sin(theta); % Y-axis points for the first circle
z1 = zeros(1, 100); % Z-axis points for the first circle

% Setup input joint angles for animation
x_vals = linspace(-50, 50, 50); % Change in X
y_vals = linspace(-50, 50, 50); % Change in Y
z_vals = linspace(-200, -100, 50); % Change in Z

while true
    % Move up and down
    for z = z_vals
        move_robot(0, 0, z, R, L, r, theta, x1, y1, z1);
    end
    for z = flip(z_vals)
        move_robot(0, 0, z, R, L, r, theta, x1, y1, z1);
    end
    % Move right and left
    for x = x_vals
        move_robot(x, 0, z_vals(1), R, L, r, theta, x1, y1, z1);
    end
    for x = flip(x_vals)
        move_robot(x, 0, z_vals(1), R, L, r, theta, x1, y1, z1);
    end
    % Move forward and backward
    for y = y_vals
        move_robot(0, y, z_vals(1), R, L, r, theta, x1, y1, z1);
    end
    for y = flip(y_vals)
        move_robot(0, y, z_vals(1), R, L, r, theta, x1, y1, z1);
    end

    % Move in circular path in XY plane
    for i = 1:length(theta)
        move_robot(25 * cos(theta(i)), 25 * sin(theta(i)), z_vals(1), R, L, r, theta, x1, y1, z1);
    end
end

function move_robot(X, Y, Z, R, L, r, theta, x1, y1, z1)
    figure(1);
    clf;
    hold on;
    grid on;
    axis equal;
    xlim([-300 300]);
    ylim([-300 300]);
    zlim([-300 300]);
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
    title('3D Inverse Kinematics');
    view(3);

    % Plot the first circle in 3D
    plot3(x1, y1, z1, 'LineWidth', 2);

    try
        % Calculate joint angles using inverse kinematics
        [th1, th2, th3, fl] = IKinem(X, Y, Z);
        if fl == -1
            disp(['Invalid value for position: X=', num2str(X), ...
                  ', Y=', num2str(Y), ', Z=', num2str(Z)]);
            return;
        end
        
        th = [th1, th2, th3];
        
        % Base points
        A1 = [R*cosd(0), R*sind(0), 0];
        A2 = [R*cosd(120), R*sind(120), 0];
        A3 = [R*cosd(240), R*sind(240), 0];
        
        % Links positions
        B1 = A1 + [L*cosd(th(1)), L*sind(th(1)), 0];
        B2 = A2 + [L*cosd(th(2) + 120), L*sind(th(2) + 120), 0];
        B3 = A3 + [L*cosd(th(3) + 240), L*sind(th(3) + 240), 0];

        % Define points C1, C2, C3 as tangent points of the second circle
        C1 = [X + r*cosd(0), Y + r*sind(0), Z];
        C2 = [X + r*cosd(120), Y + r*sind(120), Z];
        C3 = [X + r*cosd(240), Y + r*sind(240), Z];

        circle2X = X + r * cos(theta);
        circle2Y = Y + r * sin(theta);
        circle2Z = Z * ones(1, length(theta));

        hold on;
        plot3([A1(1) B1(1)], [A1(2) B1(2)], [A1(3) B1(3)], 'k', 'LineWidth', 2);
        plot3([A2(1) B2(1)], [A2(2) B2(2)], [A2(3) B2(3)], 'k', 'LineWidth', 2);
        plot3([A3(1) B3(1)], [A3(2) B3(2)], [A3(3) B3(3)], 'k', 'LineWidth', 2);

        % Plot lines from B points to C points
        plot3([B1(1) C1(1)], [B1(2) C1(2)], [B1(3) C1(3)], 'k', 'LineWidth', 2);
        plot3([B2(1) C2(1)], [B2(2) C2(2)], [B2(3) C2(3)], 'k', 'LineWidth', 2);
        plot3([B3(1) C3(1)], [B3(2) C3(2)], [B3(3) C3(3)], 'k', 'LineWidth', 2);

        % Plot points C1, C2, C3
        plot3(C1(1), C1(2), C1(3), 'ro', 'MarkerSize', 10); % Plot point C1
        plot3(C2(1), C2(2), C2(3), 'go', 'MarkerSize', 10); % Plot point C2
        plot3(C3(1), C3(2), C3(3), 'bo', 'MarkerSize', 10); % Plot point C3

        % Plot the moving circle centered at [X, Y, Z]
        plot3(circle2X, circle2Y, circle2Z, 'LineWidth', 2);
        plot3(X, Y, Z, 'o', 'MarkerSize', 10);

    catch ME
        disp(['No valid solution found for position: X=', num2str(X), ...
              ', Y=', num2str(Y), ', Z=', num2str(Z)]);
        disp(ME.message);
    end

    pause(0.005);
end