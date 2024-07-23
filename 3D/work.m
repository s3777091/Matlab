close all;
clc;
clear;

% Delta robot constants
global R r L l
R=120;
r=30;
L=90;
l=250;
% Setup input joint angles
angle = linspace(0, pi/2, 25);
% ndgrid is used to obtain all possible combinations
[th1, th2, th3] = ndgrid(angle, angle, angle);

% Initialize workspace variable
wrkspace = [];

% Sweep through all possible angles
for i = 1:numel(th1)
    % Solve using forward kinematics
    [x, y, z] = FKinem(rad2deg(th1(i)), rad2deg(th2(i)), rad2deg(th3(i)));
    % If a solution exists, the point is part of the workspace
    if (isreal(x) && isreal(y) && isreal(z))
        sola = [x(1), y(1), z(1)];
        % Check if second solution exists
        if length(x) > 1
            solb = [x(2), y(2), z(2)];
            if (solb(3) < 0)
                wrkspace = cat(1, wrkspace, solb);
            end
        else
            % Only one solution, check if it is valid
            if (sola(3) < 0)
                wrkspace = cat(1, wrkspace, sola);
            end
        end
    end
end

% Convert workspace to double for saving to file
wrkspace = double(wrkspace);

% Check if workspace is not empty before plotting
if ~isempty(wrkspace)
    % Plot smooth workspace from alphaShape
    figure
    x = wrkspace(:,1);
    y = wrkspace(:,2);
    z = wrkspace(:,3);
    shp = alphaShape(x, y, z);
    plot(shp)
    title('Delta Robot Workspace')
    xlabel('x (mm)')
    ylabel('y (mm)')
    zlabel('z (mm)')
    pbaspect([1 1 1])

    % Obtain workspace volume
    V = volume(shp);
    fprintf('Workspace Volume: %.2f cubic mm\n', V);
else
    fprintf('No valid points found in the workspace.\n');
end