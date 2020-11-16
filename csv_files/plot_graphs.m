% clear all
close all
clc

% odometry_path = readtable('robot_path_path.csv', 'HeaderLines', 1);
% ground_truth_path = readtable('ground_truth_path_path.csv', 'HeaderLines', 1);
% control_states = readtable('control_states_path.csv', 'HeaderLines', 1);
% robot_velocities = readtable('robot_velocities_path.csv', 'HeaderLines', 1);


x_odom = odometry_path.Var1;
y_odom = odometry_path.Var2;

x_gt = ground_truth_path.Var1;
y_gt = ground_truth_path.Var2;

dist_r = control_states.Var1;
sigma = control_states.Var2;
theta = control_states.Var3;

angular_vel_ground_truth = robot_velocities.Var1;
angular_vel_odometry = robot_velocities.Var2;
left_wheel_angular_ground_truth = robot_velocities.Var3;
left_wheel_angular_odometry = robot_velocities.Var4;
linear_vel_ground_truth = robot_velocities.Var5;
linear_vel_odometry = robot_velocities.Var6;
right_wheel_angular_ground_truth = robot_velocities.Var7;
right_wheel_angular_odometry = robot_velocities.Var8;

time = robot_velocities.Var9;

%% Posições
poses = [[1, 1, pi/2]; [3, 1, 0]; [3, -3, -2.36]; [-3, -3, pi/2]; [-3, 0, pi/2]; [0, 0, 0]];

%% Gráfico da velocidade angular das rodas
% figure(1)
% subplot(2, 1, 1)
% plot(time, left_wheel_angular_ground_truth, 'k')
% hold on
% plot(time, left_wheel_angular_odometry, '--r')
% ylabel("Vel. angular da roda esquerda (rad/s)")
% xlabel("Tempo (s)")
% set(gca,'XLim', [0 time(end)+0.2])
% set(gca,'YLim', [min(left_wheel_angular_ground_truth)-0.1 max(left_wheel_angular_ground_truth)+0.1])
% legend("Ground Truth", "Odometria")
% grid on
% 
% subplot(2, 1, 2)
% plot(time, right_wheel_angular_ground_truth, 'k')
% hold on
% plot(time, right_wheel_angular_odometry, '--r')
% ylabel("Vel. angular da roda esquerda (rad/s)")
% xlabel("Tempo (s)")
% set(gca,'XLim', [0 time(end)+0.2])
% set(gca,'YLim', [min(right_wheel_angular_odometry)-0.1 max(right_wheel_angular_odometry)+0.1])
% grid on
% legend("Ground Truth", "Odometria")

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

%% Gráfico das velocidades angulares do robô
% figure(3)
% subplot(2, 1, 1)
% plot(time, linear_vel_ground_truth, 'k')
% hold on
% plot(time, linear_vel_odometry, '--r')
% ylabel("Vel. Linear (m/s)")
% xlabel("Tempo (s)")
% set(gca,'XLim', [0 time(end)+0.2])
% set(gca, 'XTick', 0 : 5 : time(end)+5)
% set(gca,'YLim', [min(linear_vel_ground_truth)-0.05 max(linear_vel_ground_truth)+0.1])
% set(gca, 'YTick', round(min(linear_vel_ground_truth)-0.05, 1) : 0.1 : max(linear_vel_ground_truth)+0.1)
% legend("Ground Truth", "Odometria")
% grid on
% 
% subplot(2, 1, 2)
% plot(time, angular_vel_ground_truth, 'k')
% hold on
% plot(time, angular_vel_odometry, '--r')
% ylabel("Vel. Angular (rad/s)")
% xlabel("Tempo (s)")
% set(gca,'XLim', [0 time(end)+0.2])
% set(gca, 'XTick', 0:5:time(end)+5)
% set(gca,'YLim', [min(angular_vel_ground_truth)-0.1 max(angular_vel_ground_truth)+0.1])
% set(gca, 'YTick', round(min(angular_vel_ground_truth)-0.1, 1) : 0.2 : max(angular_vel_ground_truth)+0.1)
% legend("Ground Truth", "Odometria")
% grid on

%% Control states
% figure(4)
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