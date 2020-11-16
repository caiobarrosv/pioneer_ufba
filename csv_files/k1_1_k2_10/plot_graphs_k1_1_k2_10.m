clear all
close all
clc

pose_1_gt_10 = readtable('ground_truth_path_[0, 5, 0]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_2_gt_10 = readtable('ground_truth_path_[3.54, 3.54, 0]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_3_gt_10 = readtable('ground_truth_path_[5, 0, 0]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_4_gt_10 = readtable('ground_truth_path_[4.5, -2.17, -0.785]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_5_gt_10 = readtable('ground_truth_path_[2.5, -4.33, -1.57]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_6_gt_10 = readtable('ground_truth_path_[0, -5, -1.57]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_7_gt_10 = readtable('ground_truth_path_[-3.54, -3.54, -2.36]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_8_gt_10 = readtable('ground_truth_path_[-5, 0, -3.14]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_9_gt_10 = readtable('ground_truth_path_[-3.54, 3.54, 0]_k1_1_k2_10.csv', 'HeaderLines', 1);

pose_1_odometro_10 = readtable('robot_path_[0, 5, 0]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_2_odometro_10 = readtable('robot_path_[3.54, 3.54, 0]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_3_odometro_10 = readtable('robot_path_[5, 0, 0]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_4_odometro_10 = readtable('robot_path_[4.5, -2.17, -0.785]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_5_odometro_10 = readtable('robot_path_[2.5, -4.33, -1.57]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_6_odometro_10 = readtable('robot_path_[0, -5, -1.57]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_7_odometro_10 = readtable('robot_path_[-3.54, -3.54, -2.36]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_8_odometro_10 = readtable('robot_path_[-5, 0, -3.14]_k1_1_k2_10.csv', 'HeaderLines', 1);
pose_9_odometro_10 = readtable('robot_path_[-3.54, 3.54, 0]_k1_1_k2_10.csv', 'HeaderLines', 1);

poses = [[0, 5, 0]; [3.54, 3.54, 0]; [5, 0, 0]; [4.5, -2.17, -0.785]; [2.5, -4.33, -1.57]; [0, -5, -1.57];
         [-3.54, -3.54, -2.36]; [-5, 0, -3.14]; [-3.54, 3.54, 0]];

DrawRobot([pose_1_gt_10.Var1(end), pose_1_gt_10.Var2(end), poses(1, 3)], 'k')
hold on
DrawRobot([pose_2_gt_10.Var1(end), pose_2_gt_10.Var2(end), poses(2, 3)], 'k')
hold on
DrawRobot([pose_3_gt_10.Var1(end), pose_3_gt_10.Var2(end), poses(3, 3)], 'k')
hold on
DrawRobot([pose_4_gt_10.Var1(end), pose_4_gt_10.Var2(end), poses(4, 3)], 'k')
hold on
DrawRobot([pose_5_gt_10.Var1(end), pose_5_gt_10.Var2(end), poses(5, 3)], 'k')
hold on
DrawRobot([pose_6_gt_10.Var1(end), pose_6_gt_10.Var2(end), poses(6, 3)], 'k')
hold on
DrawRobot([pose_7_gt_10.Var1(end), pose_7_gt_10.Var2(end), poses(7, 3)], 'k')
hold on
DrawRobot([pose_8_gt_10.Var1(end), pose_8_gt_10.Var2(end), poses(8, 3)], 'k')
hold on
DrawRobot([pose_9_gt_10.Var1(end), pose_9_gt_10.Var2(end), poses(9, 3)], 'k')
hold on

line_color_gt = 'k' 
plot(pose_1_gt_10.Var1, pose_1_gt_10.Var2, line_color_gt )
hold on
plot(pose_2_gt_10.Var1, pose_2_gt_10.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_3_gt_10.Var1, pose_3_gt_10.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_4_gt_10.Var1, pose_4_gt_10.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_5_gt_10.Var1, pose_5_gt_10.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_6_gt_10.Var1, pose_6_gt_10.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_7_gt_10.Var1, pose_7_gt_10.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_8_gt_10.Var1, pose_8_gt_10.Var2, line_color_gt , 'HandleVisibility', 'off')
hold on
plot(pose_9_gt_10.Var1, pose_9_gt_10.Var2, line_color_gt , 'HandleVisibility', 'off')

%% Odometria
line_color = '--r' 
hold on
plot(pose_1_odometro_10.Var1, pose_1_odometro_10.Var2, line_color)
hold on
plot(pose_2_odometro_10.Var1, pose_2_odometro_10.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_3_odometro_10.Var1, pose_3_odometro_10.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_4_odometro_10.Var1, pose_4_odometro_10.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_5_odometro_10.Var1, pose_5_odometro_10.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_6_odometro_10.Var1, pose_6_odometro_10.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_7_odometro_10.Var1, pose_7_odometro_10.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_8_odometro_10.Var1, pose_8_odometro_10.Var2, line_color, 'HandleVisibility', 'off')
hold on
plot(pose_9_odometro_10.Var1, pose_9_odometro_10.Var2, line_color, 'HandleVisibility', 'off')

grid on
xlim([-6.2 6.2])
ylim([-6.2 6])
legend('Ground Truth','Odometria')