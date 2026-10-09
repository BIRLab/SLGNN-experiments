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

delta_r_max = 200;
delta_r_min = -200;
delta_l_max = 200;
delta_l_min = -200;

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
ql = [ 0;  0;  -pi/3.7;  pi/3.0; ];       %6; 7; 8; 9; 10;
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
tspan = 0:0.01:num * T;
p_sp = p;
if p_sp == 0
    p_sp = 100;
end
p_mvn = 0;

p = p_sp;
[t_sp,y_sp]=ode15s(@bepid_VP_RNN,tspan,init,options);%ode15s much better than ode45

p = p_mvn;
[t_mvn,y_mvn]=ode15s(@bepid_VP_RNN,tspan,init,options);

% Keep the original downstream animation and legacy plots on the
% slack-enabled stair-climbing result.
p = p_sp;
t = t_sp;
y = y_sp;

%% Compare Rigid MVN and SP-VP-RNN joint velocity and slack activation
joint_velocities_sp = [y_sp(:, 11:14), y_sp(:, 16:19)];
joint_velocities_mvn = [y_mvn(:, 11:14), y_mvn(:, 16:19)];
dotq_limit_all = [dotq_r_max; dotq_l_max]';
representative_joint_ids = [4, 8];
representative_limits = dotq_limit_all(representative_joint_ids);
max_normalized_sp = max(abs(joint_velocities_sp) ./ repmat(dotq_limit_all, size(joint_velocities_sp, 1), 1), [], 2);
max_normalized_mvn = max(abs(joint_velocities_mvn) ./ repmat(dotq_limit_all, size(joint_velocities_mvn, 1), 1), [], 2);
slack_activation_r = y_sp(:, 15);
slack_activation_l = y_sp(:, 20);

fig_cmp = figure('Name', 'Stair Joint Velocity Slack Comparison', ...
    'Position', [100, 100, 760, 920], 'Color', 'w');
layout_cmp = tiledlayout(4, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
color_mvn = [0.18, 0.44, 0.62];
color_sp = [0.74, 0.30, 0.18];
color_boundary = [0.25, 0.25, 0.25];

ax_cmp(1) = nexttile;
hold on;
h_mvn_joint4 = plot(t_mvn, joint_velocities_mvn(:, representative_joint_ids(1)), ...
    'Color', color_mvn, 'LineStyle', '-', 'LineWidth', 1.35);
h_sp_joint4 = plot(t_sp, joint_velocities_sp(:, representative_joint_ids(1)), ...
    'Color', color_sp, 'LineStyle', '-', 'LineWidth', 1.35);
h_joint4_limit = yline(representative_limits(1), '--', 'Color', color_boundary, 'LineWidth', 1.1);
yline(-representative_limits(1), '--', 'Color', color_boundary, 'LineWidth', 1.1, ...
    'HandleVisibility', 'off');
box on; grid on;
xlim([tspan(1), tspan(end)]);
ylabel('Joint 4 velocity (rad/s)');
title('Stair-climbing representative joints');
legend([h_mvn_joint4, h_sp_joint4, h_joint4_limit], ...
    {'Rigid MVN', 'SP-VP-RNN', 'Velocity limit'}, ...
    'Location', 'north', 'Box', 'on');

ax_cmp(2) = nexttile;
hold on;
h_mvn_joint8 = plot(t_mvn, joint_velocities_mvn(:, representative_joint_ids(2)), ...
    'Color', color_mvn, 'LineStyle', '-', 'LineWidth', 1.35);
h_sp_joint8 = plot(t_sp, joint_velocities_sp(:, representative_joint_ids(2)), ...
    'Color', color_sp, 'LineStyle', '-', 'LineWidth', 1.35);
h_joint8_limit = yline(representative_limits(2), '--', 'Color', color_boundary, 'LineWidth', 1.1);
yline(-representative_limits(2), '--', 'Color', color_boundary, 'LineWidth', 1.1, ...
    'HandleVisibility', 'off');
box on; grid on;
xlim([tspan(1), tspan(end)]);
ylabel('Joint 8 velocity (rad/s)');
legend([h_mvn_joint8, h_sp_joint8, h_joint8_limit], ...
    {'Rigid MVN', 'SP-VP-RNN', 'Velocity limit'}, ...
    'Location', 'north', 'Box', 'on');

ax_cmp(3) = nexttile;
hold on;
h_slack_r = plot(t_sp, slack_activation_r, 'Color', color_mvn, 'LineWidth', 1.4);
h_slack_l = plot(t_sp, slack_activation_l, 'Color', color_sp, 'LineWidth', 1.4);
yline(0, '-', 'Color', [0.45, 0.45, 0.45], 'LineWidth', 0.8, ...
    'HandleVisibility', 'off');
box on; grid on;
xlim([tspan(1), tspan(end)]);
ylabel('Slack activation (mm/s)');
legend([h_slack_r, h_slack_l], {'$\dot{\delta}_R$', '$\dot{\delta}_L$'}, ...
    'Interpreter', 'latex', 'Location', 'north', 'Box', 'on');

ax_cmp(4) = nexttile;
hold on;
h_mvn_norm = plot(t_mvn, max_normalized_mvn, 'Color', color_mvn, 'LineWidth', 1.4);
h_sp_norm = plot(t_sp, max_normalized_sp, 'Color', color_sp, 'LineWidth', 1.4);
h_feasibility = yline(1, '--', 'Color', color_boundary, 'LineWidth', 1.1);
box on; grid on;
xlim([tspan(1), tspan(end)]);
ylim([0, max(1.2, 1.05 * max([max_normalized_mvn; max_normalized_sp]))]);
xlabel('Time (s)');
ylabel('Max normalized velocity');
legend([h_mvn_norm, h_sp_norm, h_feasibility], ...
    {'Rigid MVN', 'SP-VP-RNN', 'Feasibility boundary'}, ...
    'Location', 'north', 'Box', 'on');

linkaxes(ax_cmp, 'x');
set(ax_cmp, 'FontName', 'Arial', ...
    'FontSize', 9, ...
    'LineWidth', 0.8, ...
    'GridColor', [0.80, 0.80, 0.80], ...
    'GridAlpha', 0.55);
exportgraphics(fig_cmp, 'joint_velocity_slack_comparison.pdf', 'ContentType', 'vector');
exportgraphics(fig_cmp, 'joint_velocity_slack_comparison.png', 'Resolution', 600);

% Export the four comparison panels as independent figures.
for joint_panel_idx = 1:2
    joint_id = representative_joint_ids(joint_panel_idx);
    joint_limit = representative_limits(joint_panel_idx);
    fig_joint_single = figure('Name', ['Joint ', num2str(joint_id), ' Velocity Comparison'], ...
        'Position', [120, 120, 660, 300], 'Color', 'w');
    hold on;
    h_single_mvn = plot(t_mvn, joint_velocities_mvn(:, joint_id), ...
        'Color', color_mvn, 'LineWidth', 1.5);
    h_single_sp = plot(t_sp, joint_velocities_sp(:, joint_id), ...
        'Color', color_sp, 'LineWidth', 1.5);
    h_single_limit = yline(joint_limit, '--', 'Color', color_boundary, 'LineWidth', 1.1);
    yline(-joint_limit, '--', 'Color', color_boundary, 'LineWidth', 1.1, ...
        'HandleVisibility', 'off');
    box on; grid on;
    xlim([tspan(1), tspan(end)]);
    ylabel(['Joint ', num2str(joint_id), ' velocity (rad/s)']);
    xlabel('Time (s)');
    legend([h_single_mvn, h_single_sp, h_single_limit], ...
        {'Rigid MVN', 'SP-VP-RNN', 'Velocity limit'}, ...
        'Location', 'north', 'Box', 'on');
    set(gca, 'FontName', 'Arial', ...
        'FontSize', 9, ...
        'LineWidth', 0.8, ...
        'GridColor', [0.80, 0.80, 0.80], ...
        'GridAlpha', 0.55);
    exportgraphics(fig_joint_single, ['joint', num2str(joint_id), ...
        '_velocity_comparison.pdf'], 'ContentType', 'vector');
    exportgraphics(fig_joint_single, ['joint', num2str(joint_id), ...
        '_velocity_comparison.png'], 'Resolution', 600);
end

fig_slack_single = figure('Name', 'Slack Activation Comparison', ...
    'Position', [120, 120, 660, 300], 'Color', 'w');
hold on;
h_single_slack_r = plot(t_sp, slack_activation_r, 'Color', color_mvn, 'LineWidth', 1.5);
h_single_slack_l = plot(t_sp, slack_activation_l, 'Color', color_sp, 'LineWidth', 1.5);
yline(0, '-', 'Color', [0.45, 0.45, 0.45], 'LineWidth', 0.8, ...
    'HandleVisibility', 'off');
box on; grid on;
xlim([tspan(1), tspan(end)]);
ylabel('Slack activation (mm/s)');
xlabel('Time (s)');
legend([h_single_slack_r, h_single_slack_l], ...
    {'$\dot{\delta}_R$', '$\dot{\delta}_L$'}, ...
    'Interpreter', 'latex', 'Location', 'north', 'Box', 'on');
set(gca, 'FontName', 'Arial', ...
    'FontSize', 9, ...
    'LineWidth', 0.8, ...
    'GridColor', [0.80, 0.80, 0.80], ...
    'GridAlpha', 0.55);
exportgraphics(fig_slack_single, 'slack_activation_comparison.pdf', 'ContentType', 'vector');
exportgraphics(fig_slack_single, 'slack_activation_comparison.png', 'Resolution', 600);

fig_norm_single = figure('Name', 'Max Normalized Velocity Comparison', ...
    'Position', [120, 120, 660, 300], 'Color', 'w');
hold on;
h_single_norm_mvn = plot(t_mvn, max_normalized_mvn, 'Color', color_mvn, 'LineWidth', 1.5);
h_single_norm_sp = plot(t_sp, max_normalized_sp, 'Color', color_sp, 'LineWidth', 1.5);
h_single_feasibility = yline(1, '--', 'Color', color_boundary, 'LineWidth', 1.1);
box on; grid on;
xlim([tspan(1), tspan(end)]);
ylim([0, max(1.2, 1.05 * max([max_normalized_mvn; max_normalized_sp]))]);
ylabel('Max normalized velocity');
xlabel('Time (s)');
legend([h_single_norm_mvn, h_single_norm_sp, h_single_feasibility], ...
    {'Rigid MVN', 'SP-VP-RNN', 'Feasibility boundary'}, ...
    'Location', 'north', 'Box', 'on');
set(gca, 'FontName', 'Arial', ...
    'FontSize', 9, ...
    'LineWidth', 0.8, ...
    'GridColor', [0.80, 0.80, 0.80], ...
    'GridAlpha', 0.55);
exportgraphics(fig_norm_single, 'max_normalized_velocity_comparison.pdf', 'ContentType', 'vector');
exportgraphics(fig_norm_single, 'max_normalized_velocity_comparison.png', 'Resolution', 600);

%% Plot X/Y velocity-level solution error against the stair reference
% The RNN is solved in the robot body frame, while the animation later uses
% a world-frame body translation. The solver error is therefore evaluated
% as J(q)*qdot minus the same body-frame stair reference velocity used in
% bepid_VP_RNN.m.
T_trajectory_ref = 2;
L_traj_ref = 800;
h_traj_ref = 240;
dotxpos_body_ref = L_traj_ref / (T_trajectory_ref * 2);

err_x_r = zeros(size(t));
err_x_l = zeros(size(t));
err_y_r = zeros(size(t));
err_y_l = zeros(size(t));

for idx = 1:length(t)
    q_now = [y(idx, 1:4), y(idx, 6:9)]';
    dotq_now = [y(idx, 11:14), y(idx, 16:19)]';
    [~, ~, Jw_aug_now, ~] = ztm_jacobian(q_now, dotq_now);

    dotX_now = [y(idx, 11:14)'; y(idx, 15); y(idx, 16:19)'; y(idx, 20)];
    foot_velocity_now = Jw_aug_now * dotX_now;

    time_flag_ref = t(idx);
    while time_flag_ref > T_trajectory_ref
        time_flag_ref = time_flag_ref - T_trajectory_ref;
    end
    p_ref = time_flag_ref / T_trajectory_ref;

    if p_ref <= 0.5
        p_swing = p_ref * 2;
        dotx_r_ref = L_traj_ref / T_trajectory_ref * (1 - cos(2 * pi * p_swing));
        dotz_r_ref = pi * h_traj_ref / T_trajectory_ref * sin(2 * pi * p_swing);
        dotRw_ref = [dotx_r_ref - dotxpos_body_ref; 0; dotz_r_ref + 71; ...
                     -dotxpos_body_ref; 0; -41];
    else
        p_swing = (p_ref - 0.5) * 2;
        dotx_l_ref = L_traj_ref / T_trajectory_ref * (1 - cos(2 * pi * p_swing));
        dotz_l_ref = pi * h_traj_ref / T_trajectory_ref * sin(2 * pi * p_swing);
        dotRw_ref = [-dotxpos_body_ref; 0; -40; ...
                     dotx_l_ref - dotxpos_body_ref; 0; dotz_l_ref + 40];
    end

    velocity_error_now = foot_velocity_now - dotRw_ref;
    err_x_r(idx) = velocity_error_now(1);
    err_y_r(idx) = velocity_error_now(2);
    err_x_l(idx) = velocity_error_now(4);
    err_y_l(idx) = velocity_error_now(5);
end

err_x_r_m = err_x_r / 1000;
err_y_r_m = err_y_r / 1000;
err_x_l_m = err_x_l / 1000;
err_y_l_m = err_y_l / 1000;
err_norm_r_m = sqrt(err_x_r_m.^2 + err_y_r_m.^2);
err_norm_l_m = sqrt(err_x_l_m.^2 + err_y_l_m.^2);

stable_error_start_time = 7;
stable_idx = t >= stable_error_start_time;
if ~any(stable_idx)
    stable_idx = true(size(t));
end

err_colors = [0.80, 0.22, 0.13; ...
              0.10, 0.55, 0.25; ...
              0.08, 0.08, 0.08];
leg_names = {'Right leg', 'Left leg'};
leg_tags = {'right_leg', 'left_leg'};
err_x_all = {err_x_r_m, err_x_l_m};
err_y_all = {err_y_r_m, err_y_l_m};
err_norm_all = {err_norm_r_m, err_norm_l_m};

for leg_idx = 1:2
    fig_err = figure('Name', ['XY Solution Error - ', leg_names{leg_idx}], ...
        'Position', [100, 100, 520, 360], 'Color', 'w');
    hold on;
    plot(t, err_x_all{leg_idx}, 'Color', err_colors(1,:), 'LineWidth', 1.4);
    plot(t, err_y_all{leg_idx}, 'Color', err_colors(2,:), 'LineWidth', 1.4);
    plot(t, err_norm_all{leg_idx}, 'Color', err_colors(3,:), 'LineWidth', 1.6);
    yline(0, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 0.9);
    box on; grid on;
    xlim([t(1), t(end)]);
    legend({'$e_x$', '$e_y$', '$\|e\|$'}, ...
        'Interpreter', 'latex', 'Location', 'northeast', 'Box', 'off');
    text(0.02, 0.94, 'error / m s^{-1}', 'Units', 'normalized', ...
        'FontName', 'Times New Roman', 'FontSize', 12);
    text(0.78, 0.06, 'time / s', 'Units', 'normalized', ...
        'FontName', 'Times New Roman', 'FontSize', 12);
    text(0.04, 0.08, leg_names{leg_idx}, 'Units', 'normalized', ...
        'FontName', 'Times New Roman', 'FontSize', 11);
    set(gca, 'FontName', 'Times New Roman', ...
        'FontSize', 10, ...
        'LineWidth', 0.9, ...
        'GridColor', [0.82 0.85 0.88], ...
        'GridAlpha', 0.55);
    exportgraphics(fig_err, ['xy_solution_error_', leg_tags{leg_idx}, '.pdf'], ...
        'ContentType', 'vector');
    exportgraphics(fig_err, ['xy_solution_error_', leg_tags{leg_idx}, '.png'], ...
        'Resolution', 600);

    fig_stable = figure('Name', ['Stable XY Solution Error - ', leg_names{leg_idx}], ...
        'Position', [140, 140, 520, 360], 'Color', 'w');
    hold on;
    plot(t(stable_idx), err_x_all{leg_idx}(stable_idx), ...
        'Color', err_colors(1,:), 'LineWidth', 1.4);
    plot(t(stable_idx), err_y_all{leg_idx}(stable_idx), ...
        'Color', err_colors(2,:), 'LineWidth', 1.4);
    plot(t(stable_idx), err_norm_all{leg_idx}(stable_idx), ...
        'Color', err_colors(3,:), 'LineWidth', 1.6);
    yline(0, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 0.9);
    box on; grid on;
    xlim([t(find(stable_idx, 1, 'first')), t(end)]);
    legend({'$e_x$', '$e_y$', '$\|e\|$'}, ...
        'Interpreter', 'latex', 'Location', 'northeast', 'Box', 'off');
    text(0.02, 0.94, 'error / m s^{-1}', 'Units', 'normalized', ...
        'FontName', 'Times New Roman', 'FontSize', 12);
    text(0.68, 0.06, 'time / s', 'Units', 'normalized', ...
        'FontName', 'Times New Roman', 'FontSize', 12);
    text(0.04, 0.08, [leg_names{leg_idx}, ', t \geq ', ...
        num2str(stable_error_start_time), ' s'], ...
        'Units', 'normalized', 'Interpreter', 'tex', ...
        'FontName', 'Times New Roman', 'FontSize', 11);
    set(gca, 'FontName', 'Times New Roman', ...
        'FontSize', 10, ...
        'LineWidth', 0.9, ...
        'GridColor', [0.82 0.85 0.88], ...
        'GridAlpha', 0.55);
    exportgraphics(fig_stable, ['xy_solution_error_', leg_tags{leg_idx}, ...
        '_stable_zoom.pdf'], 'ContentType', 'vector');
    exportgraphics(fig_stable, ['xy_solution_error_', leg_tags{leg_idx}, ...
        '_stable_zoom.png'], 'Resolution', 600);

    fig_inset = figure('Name', ['XY Solution Error with Inset - ', leg_names{leg_idx}], ...
        'Position', [180, 180, 620, 420], 'Color', 'w');
    main_ax = axes('Parent', fig_inset, 'Position', [0.11, 0.13, 0.82, 0.78]);
    hold(main_ax, 'on');
    plot(main_ax, t, err_x_all{leg_idx}, 'Color', err_colors(1,:), 'LineWidth', 1.35);
    plot(main_ax, t, err_y_all{leg_idx}, 'Color', err_colors(2,:), 'LineWidth', 1.35);
    plot(main_ax, t, err_norm_all{leg_idx}, 'Color', err_colors(3,:), 'LineWidth', 1.55);
    yline(main_ax, 0, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 0.85);
    xline(main_ax, stable_error_start_time, ':', 'Color', [0.35 0.35 0.35], ...
        'LineWidth', 0.9, 'HandleVisibility', 'off');
    box(main_ax, 'on');
    grid(main_ax, 'on');
    xlim(main_ax, [t(1), t(end)]);
    legend(main_ax, {'$e_x$', '$e_y$', '$\|e\|$'}, ...
        'Interpreter', 'latex', 'Location', 'northeast', 'Box', 'off');
    text(main_ax, 0.02, 0.94, 'error / m s^{-1}', 'Units', 'normalized', ...
        'FontName', 'Times New Roman', 'FontSize', 12);
    text(main_ax, 0.78, 0.06, 'time / s', 'Units', 'normalized', ...
        'FontName', 'Times New Roman', 'FontSize', 12);
    text(main_ax, 0.04, 0.08, leg_names{leg_idx}, 'Units', 'normalized', ...
        'FontName', 'Times New Roman', 'FontSize', 11);

    inset_ax = axes('Parent', fig_inset, 'Position', [0.54, 0.54, 0.33, 0.27]);
    hold(inset_ax, 'on');
    plot(inset_ax, t(stable_idx), err_x_all{leg_idx}(stable_idx), ...
        'Color', err_colors(1,:), 'LineWidth', 1.05);
    plot(inset_ax, t(stable_idx), err_y_all{leg_idx}(stable_idx), ...
        'Color', err_colors(2,:), 'LineWidth', 1.05);
    plot(inset_ax, t(stable_idx), err_norm_all{leg_idx}(stable_idx), ...
        'Color', err_colors(3,:), 'LineWidth', 1.20);
    yline(inset_ax, 0, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 0.75);
    box(inset_ax, 'on');
    grid(inset_ax, 'on');
    xlim(inset_ax, [t(find(stable_idx, 1, 'first')), t(end)]);
    title(inset_ax, ['t \geq ', num2str(stable_error_start_time), ' s'], ...
        'Interpreter', 'tex', 'FontWeight', 'normal', 'FontSize', 8);

    set([main_ax, inset_ax], ...
        'FontName', 'Times New Roman', ...
        'LineWidth', 0.9, ...
        'GridColor', [0.82 0.85 0.88], ...
        'GridAlpha', 0.55);
    set(main_ax, 'FontSize', 10);
    set(inset_ax, 'FontSize', 7);
    exportgraphics(fig_inset, ['xy_solution_error_', leg_tags{leg_idx}, ...
        '_with_inset.pdf'], 'ContentType', 'vector');
    exportgraphics(fig_inset, ['xy_solution_error_', leg_tags{leg_idx}, ...
        '_with_inset.png'], 'Resolution', 600);
end

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

% ------------------- 绘制楼梯 ----------------------
% -------- 连贯楼梯绘制 --------
num_steps = 8;            % 楼梯阶数
step_width = 380;          % 每阶长度（x方向）
step_height = 80;          % 每阶高度（z方向）
step_depth = 600;          % 每阶深度（y方向）

start_x = -200;            % 楼梯起始 x 坐标
start_z = -640;            % 楼梯起始 z 坐标
color = [0.6, 0.6, 0.6];   % 灰色

for i = 0:num_steps-1
    x0 = start_x + i * step_width;
    z0 = start_z + i * step_height;

    % 顶部面（水平）
    x_top = [0, step_width, step_width, 0] + x0;
    y_top = [-step_depth/2, -step_depth/2, step_depth/2, step_depth/2];
    z_top = [0, 0, 0, 0] + z0;
    fill3(x_top, y_top, z_top, color, 'EdgeColor', 'none');

    % 前侧面（垂直）
    if i > 0  % 第一个台阶不需要前面
        x_front = [0, step_width, step_width, 0] + x0;
        y_front = [-step_depth/2, -step_depth/2, -step_depth/2, -step_depth/2];
        z_front = [0, 0, -step_height, -step_height] + z0;
        fill3(x_front, y_front, z_front, color * 0.8, 'EdgeColor', 'none');
    end
    % 左侧面（只画左边缘那一面）
    if i > 0  % 第一个台阶不需要前面
    x_left = [0, 0, 0, 0] + x0;
    y_left = [-step_depth/2, -step_depth/2, step_depth/2, step_depth/2];
    z_left = [0, -step_height, -step_height, 0] + z0;
    fill3(x_left, y_left, z_left, color * 0.7, 'EdgeColor', 'none');
    end
end


V_body_x = 200;%身体前进速度
% 身体沿着楼梯走，高度随前进增加
x_pos = V_body_x * (t_now/100);
z_pos = step_height/2 * (t_now/100);

ch = [x_pos, 0, z_pos];

    
    qr = [y(t_now, 1:4)'; -y(t_now,3) - y(t_now,4)];%计算脚踝关节为水平
    ql = [y(t_now, 6:9)'; -y(t_now,8) - y(t_now,9)];
    q = [0; qr; ql];

    [xr, yr, zr, xl, yl, zl, ~] = ztmkinematics(l1, l2, l3, l4, l5, l6, q);

    % 添加当前末端轨迹点
foot_r_end = [xr(end-1) + ch(1), yr(end-1), zr(end-1) + ch(3)];
foot_l_end = [xl(end-1) + ch(1), yl(end-1), zl(end-1) + ch(3)];

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
%legend('身体中心点','右腿', '左腿', '躯干', '头部', '右臂', '左臂');
saveas(gcf, 'robot_animation.png');

%% 截取目标视角图片
% 1. 设置视角
view(-27.5, 20); 

% 2. 自动调整坐标轴比例，确保人体不被拉伸
axis equal; 

% 3. 让坐标轴紧凑显示，减少留白
axis tight; 

% 4. 这里的 view 可能需要微调，如果想和原图完全一样，可以尝试：
% view(120, 20); % 如果左右反了，尝试这个

% 5. 导出高质量图片
exportgraphics(gcf, 'robot_climb_stair_target_view.png', 'Resolution', 300);
%% 绘制角速度图
% 提取关节角速度信息
joint_velocities = [y(:, 11:14), y(:, 16:19)]; 

% 添加一个总体标题
% sgtitle('8个关节的角速度关于时间的图像');
plot_individual_joint_figures = false;
for i = 1:(8 * double(plot_individual_joint_figures))
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
figure('Name', 'Joint Velocity Combined', 'Position', [100, 100, 1000, 800], 'Color', 'w');
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

% 总体标题和轴标签
sgtitle('Joint Velocities with Limits', 'FontWeight', 'bold', 'FontSize', 14);
xlabel(layout, 'Time (s)', 'FontSize', 12);
ylabel(layout, 'Joint Velocity (rad/s)', 'FontSize', 12);

% 导出图像
exportgraphics(gcf, 'joint_velocity_combined.png', 'Resolution', 600);

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

% 绘制上下限线（这里取全局限幅）
yline(max(dotq_r_max), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
yline(min(dotq_r_min), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);

% 标签与标题
xlabel('Time (s)', 'FontSize', 12);
ylabel('Joint Velocity (rad/s)', 'FontSize', 12);
title('All Joint Velocities with Limits', 'FontWeight', 'bold', 'FontSize', 14);
legend({'Joint 1','Joint 2','Joint 3','Joint 4','Joint 5','Joint 6','Joint 7','Joint 8',...
    'Upper/Lower Limit'}, 'Location', 'eastoutside', 'FontSize', 10);

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


%% 绘制顶刊专用图表：爬楼梯时关节速度与松弛变量的自适应权衡关系图
% 提取最容易达到速度极限的关键关节数据（通常为膝关节或髋关节，此处提取第3和第7列代表左右腿关键关节）
critical_joint_v_r = joint_velocities(:, 3); 
critical_joint_v_l = joint_velocities(:, 7); 

% 提取松弛变量（根据前面的代码，松弛变量分别在第15和20列）
slack_r = y(:, 15);
slack_l = y(:, 20);

% 创建高质感绘图窗口
figure('Name', 'Stair Climbing: Velocity vs Slack', 'Position', [100, 100, 850, 600]);

% ---------------- 子图 (a): 关节速度与物理极限 ----------------
ax1 = subplot(2,1,1);
plot(t, critical_joint_v_r, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.8); hold on; % 经典的 MATLAB 蓝
plot(t, critical_joint_v_l, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 1.8); % 经典的 MATLAB 绿
% 绘制 1.8 rad/s 的速度极限红线
yline(1.8, '--', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);
yline(-1.8, '--', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);

% 设置范围与标签
ylim([-2.5, 2.5]);
xlim([t(1), t(end)]);
grid on;
ylabel('Joint Velocity (rad/s)', 'FontSize', 12, 'FontName', 'Times New Roman');
title('(a) Critical Joint Velocities Reaching Physical Limits', 'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
legend('Right Leg Critical Joint', 'Left Leg Critical Joint', 'Velocity Limit (\pm 1.8)', 'Location', 'northeast', 'FontSize', 10, 'FontName', 'Times New Roman');

% ---------------- 子图 (b): 松弛变量的自适应激活 ----------------
ax2 = subplot(2,1,2);
plot(t, slack_r, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.8); hold on;
plot(t, slack_l, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 1.8);

% 设置范围与标签
xlim([t(1), t(end)]);
grid on;
xlabel('Time (s)', 'FontSize', 12, 'FontName', 'Times New Roman');
ylabel('Slack Variable \delta', 'FontSize', 12, 'FontName', 'Times New Roman');
title('(b) Adaptive Activation of Trajectory Slack Variables', 'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
legend('Right Leg \delta_R', 'Left Leg \delta_L', 'Location', 'northeast', 'FontSize', 10, 'FontName', 'Times New Roman');

% 调整整体布局，减少子图之间的留白
set(ax1, 'Position', [0.1, 0.55, 0.85, 0.35]);
set(ax2, 'Position', [0.1, 0.12, 0.85, 0.35]);

% 导出为 600 DPI 的高质量矢量/位图，供 IEEE 论文直接使用
exportgraphics(gcf, 'Stair_Climbing_Velocity_vs_Slack.png', 'Resolution', 600);

%% 绘制顶刊专属图：关键关节速度(Joint 4 & 8)与松弛变量的自适应权衡机制

% 提取最容易达到速度极限的关键关节数据（第 4 和第 8 列）
critical_joint_v_r = joint_velocities(:, 4); % 右腿关键关节 (Joint 4)
critical_joint_v_l = joint_velocities(:, 8); % 左腿关键关节 (Joint 8)

% 提取松弛变量（根据前面的代码，松弛变量分别在第15和20列）
slack_r = y(:, 15);
slack_l = y(:, 20);

% 创建高质感绘图窗口
figure('Name', 'Stair Climbing: Velocity vs Slack', 'Position', [100, 100, 850, 600]);

% ---------------- 子图 (a): 关节速度与物理极限 ----------------
ax1 = subplot(2,1,1);
% 使用顶刊经典的对比配色：学术蓝 (Blue) 与 赤土红 (Brick Red)
plot(t, critical_joint_v_r, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.8); hold on; 
plot(t, critical_joint_v_l, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.8); 

% 绘制 1.8 rad/s 的速度极限边界 (深灰色粗虚线)
yline(1.8, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5);
yline(-1.8, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5);

% 设置范围与标签
ylim([-2.5, 2.5]);
xlim([t(1), t(end)]);
grid on; box on;
ylabel('Joint Velocity (rad/s)', 'FontSize', 12, 'FontName', 'Times New Roman');
title('(a) Velocities of Critical Joints (Joint 4 & 8) Reaching Physical Limits', 'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
legend('Joint 4 (Right Leg)', 'Joint 8 (Left Leg)', 'Velocity Limit (\pm1.8)', 'Location', 'northeast', 'FontSize', 10, 'FontName', 'Times New Roman');

% ---------------- 子图 (b): 松弛变量的自适应激活 ----------------
ax2 = subplot(2,1,2);
plot(t, slack_r, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.8); hold on;
plot(t, slack_l, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.8);

% 绘制 0 刻度基准线
yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);

% 设置范围与标签
xlim([t(1), t(end)]);
grid on; box on;
xlabel('Time (s)', 'FontSize', 12, 'FontName', 'Times New Roman');
ylabel('Slack Variable \delta', 'FontSize', 12, 'FontName', 'Times New Roman');
title('(b) Adaptive Activation of Trajectory Slack Variables', 'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
legend('Right Leg \delta_R', 'Left Leg \delta_L', 'Location', 'northeast', 'FontSize', 10, 'FontName', 'Times New Roman');

% 调整整体布局，减少子图之间的留白，使得导出到论文时更加紧凑
set(ax1, 'Position', [0.1, 0.53, 0.85, 0.37]);
set(ax2, 'Position', [0.1, 0.10, 0.85, 0.37]);

% 导出为 600 DPI 的高质量图片，供 IEEE 论文直接使用
exportgraphics(gcf, 'Stair_Climbing_Velocity_vs_Slack.png', 'Resolution', 600);
