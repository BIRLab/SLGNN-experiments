%% 定义基本参数  计算初值

clc;
close all;
clear all;
format long;

global  n m k p sigma gama dotX_max dotX_min q_max q_min;  
global l1 l2 l3 l4 l5 l6;
% global  qr; ql; q_0; 


%% 系统参数

%矩阵大小
n = 10;  m = 10;  k = 6;%n为x变量数，此处为总自由度 + ；m个人理解为不等式约束项变量个数; k可理解为等式约束变量(列的)数量
p = 100;  num_lamuda = 6;    %惩罚因子（不等式约束）  以及拉格朗日乘子λ维数
sigma = 20; %惩罚函数系数
gama = 100; %惩罚函数系数gama

%机器人各个杆长度
l1=90; l2=0; l3=10; l4=370; l5=310; l6=100;%腿部杆长

%速度上下限制
q_min_va = -1.8;
q_max_va = 1.8;

 dotq_r_max = [3.4323; 2.7; 1.8; 1.8];
 dotq_r_min = dotq_r_max * -1;
 
 dotq_l_max = [3.4323; 2.7; 1.8; 1.8];
 dotq_l_min = dotq_l_max * -1;

% dotq_r_max = [2; 2; 2; 2];
% dotq_r_min = [-2; -2; -2; -2];
% 
% dotq_l_max = [2; 2; 2; 2];
% dotq_l_min = [-2; -2; -2; -2];

delta_r_max = 100;
delta_r_min = -100;
delta_l_max = 100;
delta_l_min = -100;

dotX_max = [dotq_r_max; delta_r_max; dotq_l_max; delta_l_max];
dotX_min = [dotq_r_min; delta_r_min; dotq_l_min; delta_l_min];

%角度上下限
% q_r_max = [pi/6; pi/4; pi/6; 2*pi/3; pi/4];
% q_r_min = [-pi/6; -pi/4; -pi/2; 0; pi/4];

% q_l_max = [pi/6; pi/4; pi/6; 2*pi/3];
% q_l_min = [-pi/6; -pi/4; -pi/2; 0];
% 
% q_max = [q_r_max; q_l_max];
% q_min = [q_r_min; q_l_min];

%% 初始化参数

%q_0 = 0;  %0;
qr = [ 0;  0;  -pi/18;  pi/4.3; ];     %1; 2; 3; 4; 5（分别对应关节号）; 
ql = [ 0;  0;  -pi/6.2;  pi/5; ];       %6; 7; 8; 9; 10;
delta_r = 0;
delta_l = 0;

% qr = [ 0;  0;  0;  0; ];     %1; 2; 3; 4; 5（分别对应关节号）; 
% ql = [ 0;  0;  0;  0; ];       %6; 7; 8; 9; 10;
X_init = [qr; delta_r; ql; delta_l];

BodyPosition_init = [0;0;0];

yy0 = zeros(n + num_lamuda,1);%此处n相当于角速度数量+1

%% 求解
T = 1;
num = 10;%周期数量

init = [X_init;yy0];     %微分方程初始值

%options = odeset('MaxStep',0.05,'RelTol',1e-4,'AbsTol',1e-8*ones(size(init)));       %设置ODE相关的参数
tic;        %开启计时
options=odeset('RelTol',1e-8,'AbsTol',1e-8*ones(size(init)));
[t,y]=ode15s(@bepid_VP_RNN,0:0.01:num * T,init,options);%ode15s much better than ode45

%% 绘图

% y(:, [5, 10, 15, 20]) = [];%数据处理，删掉松弛变量

%clear all;
format long;

%l1 = 90; l2 = 0; l3 = 10; l4 = 370; l5 = 310; l6 = 100; % 腿部杆长
trunk_length = 450; % 躯干长度
head_radius = 60; % 头部半径
arm_length_1 = 100; % 双臂长度
arm_length_2 = 250;
arm_length_3 = 250;
arm_ang = pi/3;

shoulder_offset = 400; % 肩膀位置与身体中心点的垂直偏移量

% 用于记录右腿和左腿末端轨迹
trajectory_r = [];  % 右腿末端轨迹
trajectory_l = [];  % 左腿末端轨迹


figure;
    
    axis equal;
    hold on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    view(3);


num_time_points = size(t) ;


ch = [0, 0, 0]; % 身体中心点坐标
for t_now = 1:20:num_time_points(:,1)
    cla;


% 设置地面大小
[x_grand, y_grand] = meshgrid(-200:2600:2600, -500:500:500);
z_grand = -640 * ones(size(x_grand));  % 将地面设置为z = -640的平面

% 绘制地面
surf(x_grand, y_grand, z_grand);
%colormap(parula(5));  % 使用 jet colormap设计颜色





V_body_x = 200;%身体前进速度
V_body_z = 0;%身体上升速度

    ch = [V_body_x * (t_now/100),0 ,V_body_z * (t_now/100)];
    
    qr = [y(t_now, 1:4)'; -y(t_now,3) - y(t_now,4)];%计算脚踝关节为水平
    ql = [y(t_now, 6:9)'; -y(t_now,8) - y(t_now,9)];
    q = [0; qr; ql];

    [xr, yr, zr, xl, yl, zl, ~] = ztmkinematics(l1, l2, l3, l4, l5, l6, q);

    % 添加当前末端轨迹点
foot_r_end = [xr(end-1) + ch(1), yr(end-1), zr(end-1)];
foot_l_end = [xl(end-1) + ch(1), yl(end-1), zl(end-1)];

trajectory_r = [trajectory_r; foot_r_end];
trajectory_l = [trajectory_l; foot_l_end];

    % 绘制身体中心点
    scatter3(ch(1), ch(2), ch(3), 'filled', 'MarkerFaceColor', 'b');
    hold on;

    % 绘制右腿
    plot3(xr + ch(1), yr, zr + ch(3), 'r', 'LineWidth', 2);

    % 绘制左腿
    plot3(xl + ch(1), yl, zl + ch(3), 'g', 'LineWidth', 2);

    % 绘制躯干
    trunk_start = ch + [0, 0, 0];
    trunk_end = trunk_start + [0, 0, trunk_length];
    plot3([trunk_start(1), trunk_end(1)], [trunk_start(2), trunk_end(2)], [trunk_start(3), trunk_end(3)], 'k', 'LineWidth', 3);

    % 绘制头部
    head_center = trunk_end + [0, 0, head_radius];
    [x_sp, y_sp, z_sp] = sphere;
    x_head = head_radius * x_sp + head_center(1);
    y_head = head_radius * y_sp + head_center(2);
    z_head = head_radius * z_sp + head_center(3);
    surf(x_head, y_head, z_head, 'FaceColor', 'k', 'EdgeColor', 'none');

    %手臂摆动角度生成
    angle_amplitude = pi/12;%摆动幅度
    T_period = 2;%摆动周期
    arm_angles2 = angle_amplitude * sin((2 * pi * (t_now/100) / T_period) - pi/2);

    % 绘制右臂
    right_arm_start = ch + [0, 0, shoulder_offset];
    right_arm_end_1 = right_arm_start + [0, arm_length_1, 0];
    right_arm_end_2 = right_arm_end_1 - [sin(-arm_angles2) * arm_length_2, 0, cos(-arm_angles2) * arm_length_2];
    right_arm_end_3 = right_arm_end_2 + [cos(arm_ang -arm_angles2) * arm_length_3, 0, - sin(arm_ang -arm_angles2) * arm_length_3];

    plot3([right_arm_start(1), right_arm_end_1(1)], [right_arm_start(2), right_arm_end_1(2)], [right_arm_start(3), right_arm_end_1(3)], 'k', 'LineWidth', 2);
    plot3([right_arm_end_1(1), right_arm_end_2(1)], [right_arm_end_1(2), right_arm_end_2(2)], [right_arm_end_1(3), right_arm_end_2(3)], 'k', 'LineWidth', 2);
    plot3([right_arm_end_2(1), right_arm_end_3(1)], [right_arm_end_2(2), right_arm_end_3(2)], [right_arm_end_2(3), right_arm_end_3(3)], 'k', 'LineWidth', 2);
    
    
    % 绘制左臂
    left_arm_start = ch + [0, 0, shoulder_offset];
    left_arm_end = left_arm_start - [0, arm_length_1, 0];
    plot3([left_arm_start(1), left_arm_end(1)], [left_arm_start(2), left_arm_end(2)], [left_arm_start(3), left_arm_end(3)], 'c', 'LineWidth', 2);

    left_arm_start = ch + [0, 0, shoulder_offset];
    left_arm_end_1 = left_arm_start - [0, arm_length_1, 0];
    left_arm_end_2 = left_arm_end_1 - [sin(arm_angles2) * arm_length_2, 0, cos(arm_angles2) * arm_length_2];
    left_arm_end_3 = left_arm_end_2 + [cos(arm_ang +arm_angles2) * arm_length_3, 0, - sin(arm_ang +arm_angles2) * arm_length_3];
    
    plot3([left_arm_start(1), left_arm_end_1(1)], [left_arm_start(2), left_arm_end_1(2)], [left_arm_start(3), left_arm_end_1(3)], 'k', 'LineWidth', 2);
    plot3([left_arm_end_1(1), left_arm_end_2(1)], [left_arm_end_1(2), left_arm_end_2(2)], [left_arm_end_1(3), left_arm_end_2(3)], 'k', 'LineWidth', 2);
    plot3([left_arm_end_2(1), left_arm_end_3(1)], [left_arm_end_2(2), left_arm_end_3(2)], [left_arm_end_2(3), left_arm_end_3(3)], 'k', 'LineWidth', 2);
    
% 绘制右脚末端轨迹
plot3(trajectory_r(:,1), trajectory_r(:,2), trajectory_r(:,3), '--r', 'LineWidth', 1.5);

% 绘制左脚末端轨迹
plot3(trajectory_l(:,1), trajectory_l(:,2), trajectory_l(:,3), '--g', 'LineWidth', 1.5);


    drawnow;
    
    axis equal;
    hold on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    view(3);
    pause(0.1); % 适当的延迟，使得动画更加流畅
end

xlabel('X');
ylabel('Y');
zlabel('Z');
%title('10自由度双足机器人');

grid on;

view(10, 10);
%legend('身体中心点','右腿', '左腿', '躯干', '头部', '右臂', '左臂');
saveas(gcf, 'robot_animation.png');
view(0, 0); % 你想要的最终角度

exportgraphics(gcf, 'robot_simulation_final_frame.png', 'Resolution', 300);


%% 绘制角速度图
% 提取关节角速度信息
joint_velocities = [y(:, 11:14), y(:, 16:19)]; 

% 添加一个总体标题
% sgtitle('8个关节的角速度关于时间的图像');
for i = 1:8
    figure('Name', ['joint', num2str(i), '_velocity_limit'], 'Position', [100, 100, 800, 300]);
    
    % 绘制关节角速度曲线
    plot(t, joint_velocities(:, i), 'b', 'LineWidth', 2); hold on;
    
    % 绘制上下限线
    yline(q_max_va, '--r', 'LineWidth', 1.5);
    yline(q_min_va, '--r', 'LineWidth', 1.5);
    
    xlabel('time (s)');
    ylabel(['\theta_', num2str(i), ' velocity (rad/s)']);
     title(['Joint ', num2str(i), ' Velocity']);
    grid on;
    ylim([-2, 2]);
    
    %legend('Joint velocity', 'Upper limit', 'Lower limit','Location', 'northwest', 'FontSize', 10);
    
     saveas(gcf, ['joint', num2str(i), '_velocity_limit.png']);
end

% 设定时间向量（假设你已有时间变量 t）
tt = t;  % 用 tt 避免与 tiledlayout 冲突

% 设置 4x2 的子图布局
layout = tiledlayout(4, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
set(gcf, 'Position', [100, 100, 1000, 800]);  % 设置导出尺寸

for i = 1:8
    % 决定放置位置：左列是1-4，右列是5-8
    if i <= 4
        tile_row = i;
        tile_col = 1;
    else
        tile_row = i - 4;
        tile_col = 2;
    end

    % 进入对应的 tile
    nexttile((tile_row - 1) * 2 + tile_col);

    % 获取该关节角速度数据
    plot(tt, joint_velocities(:, i), 'b', 'LineWidth', 1.5); hold on;

    % 添加上下限线
    if i <= 4
        qmax = dotq_r_max(i);
        qmin = dotq_r_min(i);
    else
        qmax = dotq_l_max(i - 4);
        qmin = dotq_l_min(i - 4);
    end

    yline(qmax, '--r', 'LineWidth', 1.2);
    yline(qmin, '--r', 'LineWidth', 1.2);

    title(['Joint ', num2str(i)]);
    grid on;
    ylim([-4, 4]);
end

% % 总体标题和轴标签
% sgtitle('Joint Velocities with Limits', 'FontWeight', 'bold', 'FontSize', 14);
% xlabel(layout, 'Time (s)', 'FontSize', 12);
% ylabel(layout, 'Joint Velocity (rad/s)', 'FontSize', 12);
% 
% % 导出图像
% exportgraphics(gcf, 'joint_velocity_combined.png', 'Resolution', 600);

%% 绘制所有关节角速度在一张图中
joint_velocities = [y(:, 11:14), y(:, 16:19)]; % 右臂1-4，左臂5-8

figure('Name', 'All Joint Velocities', 'Position', [100, 100, 900, 500]);
hold on; grid on;

% 定义颜色（8个不同颜色，可根据论文风格微调）
colors = lines(8);  % MATLAB内置调色板，清晰易区分

% 绘制每个关节曲线
for i = 1:8
    plot(t, joint_velocities(:, i), 'Color', colors(i,:), 'LineWidth', 1.5);
end

% % 绘制上下限线（这里取全局限幅）
% yline(max(dotq_r_max), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
% yline(min(dotq_r_min), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);

% 标签与标题
xlabel('Time (s)', 'FontSize', 12);
ylabel('Joint Velocity (rad/s)', 'FontSize', 12);
% title('All Joint Velocities with Limits', 'FontWeight', 'bold', 'FontSize', 14);
legend({'Joint 1','Joint 2','Joint 3','Joint 4','Joint 5','Joint 6','Joint 7','Joint 8',...
    'Upper/Lower Limit'}, 'Location', 'eastoutside', 'FontSize', 10);

%     % 添加上下限线
%     if i <= 4
%         qmax = dotq_r_max(i);
%         qmin = dotq_r_min(i);
%     else
%         qmax = dotq_l_max(i - 4);
%         qmin = dotq_l_min(i - 4);
%     end

     yline(qmax, '--r', 'LineWidth', 1.2);
     yline(qmin, '--b', 'LineWidth', 1.2);

ylim([-4, 4]); % 根据实际速度范围调整
xlim([t(1), t(end)]);
box on;

% 高质量导出（适合论文）
exportgraphics(gcf, 'all_joint_velocity_with_limits.png', 'Resolution', 600);

%% 绘制松弛变量 delta_r 和 delta_l 的变化曲线

% 提取 delta_r 和 delta_l
% delta_r = y(:, 5);
% delta_l = y(:, 10);

delta_r = y(:, 15);
delta_l = y(:, 20);

% 绘制 delta_r
figure('Name', 'delta_r');
plot(t, delta_r, 'LineWidth', 2);
xlabel('time (s)');
ylabel('\delta_r ', 'Interpreter', 'tex');
title('Right leg slack variable \delta_r');
grid on;
saveas(gcf, 'delta_r_plot.pdf', 'pdf');

% 绘制 delta_l
figure('Name', 'delta_l');
plot(t, delta_l, 'LineWidth', 2);
xlabel('time (s)');
ylabel('\delta_l ', 'Interpreter', 'tex');
title('Left leg slack variable \delta_l');
grid on;
saveas(gcf, 'delta_l_plot.pdf', 'pdf');


%% Plot slack variables delta_r and delta_l (IEEE style, solid lines, cool colors)

% Extract slack variables
delta_r = y(:, 15);
delta_l = y(:, 20);

figure;
hold on;

% Define cool, low-saturation colors (RGB)
color_r = [0.00, 0.45, 0.74];   % muted blue
color_l = [0.30, 0.30, 0.30];   % dark gray (cool-neutral)

plot(t, delta_r, 'LineWidth', 1.5, 'Color', color_r);
plot(t, delta_l, 'LineWidth', 1.5, 'Color', color_l);

hold off;

% Axis labels
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Slack variable $\delta$', 'Interpreter', 'latex');

% Legend
legend({'Right leg $\delta_r$', 'Left leg $\delta_l$'}, ...
       'Interpreter', 'latex', ...
       'Location', 'best');
legend boxoff;

% Grid (light, non-distracting)
grid on;

% Axis & figure styling
set(gca, 'FontSize', 10);
set(gcf, 'Color', 'w');

% Export vector PDF
exportgraphics(gcf, 'slack_variables.pdf', 'ContentType', 'vector');
