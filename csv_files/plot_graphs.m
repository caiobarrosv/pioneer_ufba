% odometry_path = readtable('robot_path_path.csv', 'HeaderLines', 1);
% ground_truth_path = readtable('ground_truth_path_path.csv', 'HeaderLines', 1);
% control_states = readtable('control_states_path.csv', 'HeaderLines', 1);
% robot_velocities = readtable('robot_velocities_path.csv', 'HeaderLines', 1);

close all
clc

x_odom = odometry_path.Var1;
y_odom = odometry_path.Var2;

x_gt = ground_truth_path.Var1;
y_gt = ground_truth_path.Var2;

dist_r = control_states.Var1;
sigma = control_states.Var2;
theta = control_states.Var3;

angular_vel = robot_velocities.Var1;
left_wheel_angular = robot_velocities.Var2;
linear_vel = robot_velocities.Var3;
right_wheel_angular = robot_velocities.Var4;

time = robot_velocities.Var5;

%% Posições
poses = [[1, 1, pi/2]; [3, 1, 0]; [3, -3, -2.36]; [-3, -3, pi/2]; [-3, 0, pi/2]; [0, 0, 0]];

%% Gráfico da velocidade angular e linear
% figure(1)
% subplot(2, 1, 1)
% plot(time, linear_vel, 'k')
% ylabel("Vel. Linear (m/s)")
% xlabel("Tempo (s)")
% set(gca,'XLim', [0 time(end)+0.2])
% set(gca, 'XTick', 0 : 5 : time(end)+5)
% set(gca,'YLim', [min(linear_vel)-0.1 max(linear_vel)+0.1])
% set(gca, 'YTick', round(min(linear_vel)-0.1, 1) : 0.1 : max(linear_vel)+0.1)
% hold on
% 
% grid on
% 
% subplot(2, 1, 2)
% plot(time, angular_vel, 'k')
% ylabel("Vel. Angular (rad/s)")
% xlabel("Tempo (s)")
% set(gca,'XLim', [0 time(end)+0.2])
% set(gca, 'XTick', 0:5:time(end)+5)
% set(gca,'YLim', [min(angular_vel)-0.1 max(angular_vel)+0.1])
% set(gca, 'YTick', round(min(angular_vel)-0.1, 1) : 0.2 : max(angular_vel)+0.1)
% grid on

%% Gráfico do ground truth VS odometria
% figure(2)
% plot(x_odom, y_odom, 'k', 'LineWidth', 2)
% hold on
% plot(x_gt, y_gt, '--r', 'LineWidth', 2)
% grid on
% ylabel("Coordenada Y (m)")
% xlabel("Coordenada X (m)")
% for i=1:1:length(poses)
%     hold on
%     plot(poses(i, 1), poses(i, 2), 'bo', 'LineWidth', 3)
%     text(poses(i, 1) + 0.1, poses(i, 2) + 0.1, int2str(i))
% end

%% Gráfico das velocidades angulares das rodas
% figure(3)
% subplot(2, 1, 1)
% plot(time, linear_vel, 'k')
% ylabel("Vel. Linear (m/s)")
% xlabel("Tempo (s)")
% set(gca,'XLim', [0 time(end)+0.2])
% set(gca, 'XTick', 0 : 5 : time(end)+5)
% set(gca,'YLim', [min(linear_vel)-0.1 max(linear_vel)+0.1])
% set(gca, 'YTick', round(min(linear_vel)-0.1, 1) : 0.1 : max(linear_vel)+0.1)
% hold on
% 
% grid on
% 
% subplot(2, 1, 2)
% plot(time, angular_vel, 'k')
% ylabel("Vel. Angular (rad/s)")
% xlabel("Tempo (s)")
% set(gca,'XLim', [0 time(end)+0.2])
% set(gca, 'XTick', 0:5:time(end)+5)
% set(gca,'YLim', [min(angular_vel)-0.1 max(angular_vel)+0.1])
% set(gca, 'YTick', round(min(angular_vel)-0.1, 1) : 0.2 : max(angular_vel)+0.1)
% grid on

%% Control states
% figure(4)
% 
% subplot(3, 1, 1)
% plot(time, sigma, 'k')
% ylabel("\delta (rad)")
% xlabel("Tempo (s)")
% set(gca,'XLim', [0 time(end)+0.2])
% set(gca, 'XTick', 0 : 5 : time(end)+5)
% grid on
% 
% subplot(3, 1, 2)
% plot(time, theta, 'k')
% ylabel("\theta (rad)")
% xlabel("Tempo (s)")
% set(gca,'XLim', [0 time(end)+0.2])
% set(gca, 'XTick', 0:5:time(end)+5)
% grid on
% 
% subplot(3, 1, 3)
% plot(time, dist_r, 'k')
% ylabel("r (m)")
% xlabel("Tempo (s)")
% set(gca,'XLim', [0 time(end)+0.2])
% set(gca, 'XTick', 0:5:time(end)+5)
% grid on