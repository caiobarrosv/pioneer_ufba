% pose_1_gt = readtable('ground_truth_path_[0, 5, 0]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_2_gt = readtable('ground_truth_path_[3.54, 3.54, 0]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_3_gt = readtable('ground_truth_path_[5, 0, 0]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_4_gt = readtable('ground_truth_path_[4.5, -2.17, -0.785]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_5_gt = readtable('ground_truth_path_[2.5, -4.33, -1.57]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_6_gt = readtable('ground_truth_path_[0, -5, -1.57]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_7_gt = readtable('ground_truth_path_[-3.54, -3.54, -2.36]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_8_gt = readtable('ground_truth_path_[-5, 0, -3.14]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_9_gt = readtable('ground_truth_path_[-3.54, 3.54, 0]_k1_1_k2_3.csv', 'HeaderLines', 1);
% 
% pose_1_odometro = readtable('robot_path_[0, 5, 0]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_2_odometro = readtable('robot_path_[3.54, 3.54, 0]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_3_odometro = readtable('robot_path_[5, 0, 0]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_4_odometro = readtable('robot_path_[4.5, -2.17, -0.785]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_5_odometro = readtable('robot_path_[2.5, -4.33, -1.57]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_6_odometro = readtable('robot_path_[0, -5, -1.57]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_7_odometro = readtable('robot_path_[-3.54, -3.54, -2.36]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_8_odometro = readtable('robot_path_[-5, 0, -3.14]_k1_1_k2_3.csv', 'HeaderLines', 1);
% pose_9_odometro = readtable('robot_path_[-3.54, 3.54, 0]_k1_1_k2_3.csv', 'HeaderLines', 1);

close all
clc

poses = [[0, 5, 0]; [3.54, 3.54, 0]; [5, 0, 0]; [4.5, -2.17, -0.785]; [2.5, -4.33, -1.57]; [0, -5, -1.57];
         [-3.54, -3.54, -2.36]; [-5, 0, -3.14]; [-3.54, 3.54, 0]];

DrawRobot([pose_1_gt.Var1(end), pose_1_gt.Var2(end), poses(1, 3)], 'k')
hold on
DrawRobot([pose_2_gt.Var1(end), pose_2_gt.Var2(end), poses(2, 3)], 'k')
hold on
DrawRobot([pose_3_gt.Var1(end), pose_3_gt.Var2(end), poses(3, 3)], 'k')
hold on
DrawRobot([pose_4_gt.Var1(end), pose_4_gt.Var2(end), poses(4, 3)], 'k')
hold on
DrawRobot([pose_5_gt.Var1(end), pose_5_gt.Var2(end), poses(5, 3)], 'k')
hold on
DrawRobot([pose_6_gt.Var1(end), pose_6_gt.Var2(end), poses(6, 3)], 'k')
hold on
DrawRobot([pose_7_gt.Var1(end), pose_7_gt.Var2(end), poses(7, 3)], 'k')
hold on
DrawRobot([pose_8_gt.Var1(end), pose_8_gt.Var2(end), poses(8, 3)], 'k')
hold on
DrawRobot([pose_9_gt.Var1(end), pose_9_gt.Var2(end), poses(9, 3)], 'k')
hold on

line_color_gt = 'k' 
plot(pose_1_gt.Var1, pose_1_gt.Var2, line_color_gt )
hold on
plot(pose_2_gt.Var1, pose_2_gt.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_3_gt.Var1, pose_3_gt.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_4_gt.Var1, pose_4_gt.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_5_gt.Var1, pose_5_gt.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_6_gt.Var1, pose_6_gt.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_7_gt.Var1, pose_7_gt.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_8_gt.Var1, pose_8_gt.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_9_gt.Var1, pose_9_gt.Var2, line_color_gt , 'HandleVisibility', 'off')

%% Odometria
line_color = '--r' 
hold on
plot(pose_1_odometro.Var1, pose_1_odometro.Var2, line_color)
hold on
plot(pose_2_odometro.Var1, pose_2_odometro.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_3_odometro.Var1, pose_3_odometro.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_4_odometro.Var1, pose_4_odometro.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_5_odometro.Var1, pose_5_odometro.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_6_odometro.Var1, pose_6_odometro.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_7_odometro.Var1, pose_7_odometro.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_8_odometro.Var1, pose_8_odometro.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_9_odometro.Var1, pose_9_odometro.Var2, line_color, 'HandleVisibility', 'off')

grid on
xlim([-6.2 6.2])
ylim([-6.2 6])
legend('Ground Truth','Odometria')