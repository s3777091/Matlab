close all;
clc;
clear;

% Define lengths of the links
l1 = 2;
l2 = 3;
l3 = 3;
l4 = 2;
l5 = 4;

% Define time vectors
t = 0:0.05:10;
T = t(1:end-1);

% Angular velocities
om1 = 2;
om2 = 1;

% Angles
th1 = om1 * t;
th4 = om2 * t;

% Points A and E
A = [0; 0];
E = l5 * [1; 0];

% Points B and D
B = [l1 * cos(th1); l1 * sin(th1)];
D = [l5 + l4 * cos(th4); l4 * sin(th4)];

% Calculate a, b, c for the quadratic equation
a = 2 * l3 * l4 .* sin(th4) - 2 * l1 * l3 .* sin(th1);
b = 2 * l3 * l5 - 2 * l1 * l3 .* cos(th1) + 2 * l3 * l4 .* cos(th4);
c = l1^2 - l2^2 + l3^2 + l4^2 + l5^2 - 2 * l1 .* l4 .* sin(th1) .* sin(th4) - 2 * l1 .* l5 .* cos(th4) - 2 * l1 .* l4 .* cos(th1) .* cos(th4);

% Ensure arguments to atan2 are real
arg1 = real(a + sqrt(a.^2 + b.^2 - c.^2));
arg2 = real(b - c);

% Angles th3 and th2
th3 = real(2 * atan2(arg1, arg2));
th2 = real(asin((l3 * sin(th3) + l4 * sin(th4) - l1 * sin(th1)) / l2));

% Coordinates of point C
C_x = l1 * cos(th1) + l2 * cos(th2);
C_y = l1 * sin(th1) + l2 * sin(th2);
C = [C_x; C_y];

% Define the line length from point C
line_length = 5;

% Velocity and acceleration of point C
C_vx = diff(C_x) ./ diff(t);
C_vy = diff(C_y) ./ diff(t);
C_v = sqrt(C_vx.^2 + C_vy.^2);

C_ax = diff(C_vx) ./ diff(T);
C_ay = diff(C_vy) ./ diff(T);
C_a = sqrt(C_ax.^2 + C_ay.^2);

figure('Position', [50, 50, 600, 400]);

ani = subplot(4, 1, 1);
axis(ani, 'equal');
xlim(ani, [-5 10]);
ylim(ani, [-5 10]);

dis = subplot(4, 1, 2);
xlim(dis, [0 10]);
ylim(dis, [-5 5]);
xlabel(dis, 'Time (s)', 'FontSize', 8);
ylabel(dis, 'Distance wrt A', 'FontSize', 8);
hold(dis, 'on');
dis_plot = plot(dis, t, C_x, 'b');

vel = subplot(4, 1, 3);
xlim(vel, [0 10]);
ylim(vel, [0 50]);
xlabel(vel, 'Time (s)', 'FontSize', 8);
ylabel(vel, 'Velocity of C', 'FontSize', 8);
grid(vel, 'on');
hold(vel, 'on');
vel_plot = plot(vel, t(1:end-1), C_v, 'r');

acc = subplot(4, 1, 4);
xlim(acc, [0 10]);
ylim(acc, [0 600]);
xlabel(acc, 'Time (s)', 'FontSize', 8);
ylabel(acc, 'Acceleration of C', 'FontSize', 8);
grid(acc, 'on');
hold(acc, 'on');
acc_plot = plot(acc, T(1:end-1), C_a, 'g');

% Animation loop
for i = 1:length(T)
    % Animation subplot
    subplot(ani);
    cla;
    hold on;
    viscircles(A', 0.05);
    viscircles(B(:, i)', 0.05);
    viscircles(C(:, i)', 0.05);
    viscircles(D(:, i)', 0.05);
    viscircles(E', 0.05);

    line([A(1) B(1, i)], [A(2) B(2, i)]);
    line([B(1, i) C(1, i)], [B(2, i) C(2, i)]);
    line([C(1, i) D(1, i)], [C(2, i) D(2, i)]);
    line([E(1) D(1, i)], [E(2) D(2, i)]);

    % Line from point C
    line([C(1, i), C(1, i)], [C(2, i), C(2, i) + line_length], 'Color', 'k', 'LineStyle', '--');
    
    hold off;

    % Update distance plot
    set(dis_plot, 'YData', C_x(1:i));
    set(dis_plot, 'XData', t(1:i));

    % Update velocity plot
    set(vel_plot, 'YData', C_v(1:i));
    set(vel_plot, 'XData', t(1:i));

    % Update acceleration plot
    set(acc_plot, 'YData', C_a(1:i));
    set(acc_plot, 'XData', T(1:i));
    
    pause(0.005);
end