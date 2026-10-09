%% 定义基本参数  计算初值

clc;
close all;
clear all;
format long;

global  n m k p sigma gama dotX_max dotX_min q_max q_min forward_speed_scale;  
global l1 l2 l3 l4 l5 l6;
% Forward speed multiplier for this copied straight-walking experiment.
% main.m uses the world frame for visualization. bepid_VP_RNN.m uses the
% robot body frame, so it keeps the step length fixed and changes the gait
% period instead of scaling the body-frame step length.
forward_speed_scale = 1.2;
fprintf('Forward speed scale = %.3f (%.1f%% of original speed)\n', ...
    forward_speed_scale, 100 * forward_speed_scale);
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

% Keep the original downstream workflow on the slack-enabled solution.
p = p_sp;
t = t_sp;
y = y_sp;

%% Plot normalized feasible-region scan
scan_timestamp = datestr(now, 'yyyymmdd_HHMMSS');
scan_results = load_latest_mvn_baseline_scan_results();
plot_mvn_baseline_feasible_region(scan_results, scan_timestamp);

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

export_paper_quantitative_tables(scan_results, t_sp, y_sp, t_mvn, y_mvn, ...
    dotq_limit_all, forward_speed_scale, scan_timestamp);

fig_cmp = figure('Name', 'Joint Velocity Slack Comparison', ...
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
title(['Representative joints, speed scale = ', ...
    num2str(forward_speed_scale, '%.2f')]);
legend([h_mvn_joint4, h_sp_joint4, h_joint4_limit], ...
    {'Rigid MVN', 'SP-VP-RNN', 'Scaled limit'}, ...
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
    {'Rigid MVN', 'SP-VP-RNN', 'Scaled limit'}, ...
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
        {'Rigid MVN', 'SP-VP-RNN', 'Scaled limit'}, ...
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

%% Plot X/Y velocity-level solution error against the reference trajectory
% The RNN is solved in the robot body frame. Therefore the solver error is
% evaluated as J(q)*qdot minus the same body-frame cycloidal reference
% velocity used inside bepid_VP_RNN.m, instead of using the world-frame
% animation translation ch.
T_trajectory_nominal = 2;
T_trajectory_ref = T_trajectory_nominal / forward_speed_scale;
L_traj_ref = 800;
h_traj_ref = 180;
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
        dotRw_ref = [dotx_r_ref - dotxpos_body_ref; 0; dotz_r_ref; ...
                     -dotxpos_body_ref; 0; 0];
    else
        p_swing = (p_ref - 0.5) * 2;
        dotx_l_ref = L_traj_ref / T_trajectory_ref * (1 - cos(2 * pi * p_swing));
        dotz_l_ref = pi * h_traj_ref / T_trajectory_ref * sin(2 * pi * p_swing);
        dotRw_ref = [-dotxpos_body_ref; 0; 0; ...
                     dotx_l_ref - dotxpos_body_ref; 0; dotz_l_ref];
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

stable_error_start_time = 7; % Change to 6 if an earlier steady-state window is needed.
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


% 设置地面大小
[x_grand, y_grand] = meshgrid(-200:2600:2600, -500:500:500);
z_grand = -640 * ones(size(x_grand));  % 将地面设置为z = -640的平面

% 绘制地面
surf(x_grand, y_grand, z_grand);
%colormap(parula(5));  % 使用 jet colormap设计颜色





V_body_x = 200;%身体前进速度
V_body_z = 0;%身体上升速度

V_body_x = V_body_x * forward_speed_scale;
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
    T_period = T_period / forward_speed_scale;
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
plot_individual_joint_figures = false;

% 添加一个总体标题
% sgtitle('8个关节的角速度关于时间的图像');
% Disabled by default to avoid exporting eight separate joint figures.
% for i = 1:(8 * double(plot_individual_joint_figures))
%     figure('Name', ['joint', num2str(i), '_velocity_limit'], 'Position', [100, 100, 800, 300]);
%     
%     % 绘制关节角速度曲线
%     plot(t, joint_velocities(:, i), 'b', 'LineWidth', 2); hold on;
%     
%     % 绘制上下限线
%     yline(q_max_va, '--r', 'LineWidth', 1.5);
%     yline(q_min_va, '--r', 'LineWidth', 1.5);
%     
%     xlabel('time (s)');
%     ylabel(['\theta_', num2str(i), ' velocity (rad/s)']);
%      title(['Joint ', num2str(i), ' Velocity']);
%     grid on;
%     ylim([-2, 2]);
%     
%     %legend('Joint velocity', 'Upper limit', 'Lower limit','Location', 'northwest', 'FontSize', 10);
%     
%      saveas(gcf, ['joint', num2str(i), '_velocity_limit.png']);
% end

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
legend({'Joint 1','Joint 2','Joint 3','Joint 4','Joint 5','Joint 6','Joint 7','Joint 8'}, ...
    'Location', 'eastoutside', 'FontSize', 10);

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

function results = load_latest_mvn_baseline_scan_results()
files = dir('mvn_baseline_speed_scan_results_*.csv');
if isempty(files)
    error(['No mvn_baseline_speed_scan_results_*.csv file found. ', ...
        'Run run_speed_limit_scan_mvn_baseline.m once to create scan data.']);
end

[~, newest_idx] = max([files.datenum]);
results = readtable(files(newest_idx).name);
results.method_label = string(results.method_label);

if ~islogical(results.is_best)
    if isnumeric(results.is_best)
        results.is_best = results.is_best ~= 0;
    else
        results.is_best = strcmpi(string(results.is_best), "true") | ...
            strcmpi(string(results.is_best), "1");
    end
end

fprintf('Loaded feasible-region scan data: %s\n', files(newest_idx).name);
end

function plot_mvn_baseline_feasible_region(results, timestamp)
fig = figure('Name', 'MVN Baseline Feasible Region', ...
    'Position', [100, 100, 760, 460], 'Color', 'w');
hold on;
grid on;

method_labels = ["p_zero_MVN_baseline", "p_200_slack_penalty"];
legend_labels = {'MVN', 'SP-VP-RNN'};
colors = [0.18, 0.44, 0.62; 0.74, 0.30, 0.18];

best_rows = results(results.is_best == true, :);
p0_best = best_rows(best_rows.p_value == 0, :);
p200_best = best_rows(best_rows.p_value == 200, :);

region_handle = gobjects(1);
if ~isempty(p0_best) && ~isempty(p200_best)
    x_left = p0_best.speed_coeff(1);
    x_right = p200_best.speed_coeff(1);
    region_handle = patch([x_left, x_right, x_right, x_left], ...
        [0.70, 0.70, 1.00, 1.00], [0.94, 0.76, 0.48], ...
        'FaceAlpha', 0.28, 'EdgeColor', 'none', ...
        'DisplayName', 'Expanded feasible region');
end

line_handles = gobjects(1, numel(method_labels));
for idx = 1:numel(method_labels)
    rows = results(results.method_label == method_labels(idx) & ...
        results.is_best == false, :);
    [~, order] = sort(rows.speed_coeff);
    rows = rows(order, :);
    line_handles(idx) = plot(rows.speed_coeff, rows.max_normalized_qdot, '-o', ...
        'Color', colors(idx, :), ...
        'MarkerFaceColor', colors(idx, :), ...
        'MarkerSize', 4.0, ...
        'LineWidth', 1.35, ...
        'DisplayName', legend_labels{idx});
end

yline(1, '--', 'Color', [0.30, 0.30, 0.30], ...
    'LineWidth', 1.0, 'HandleVisibility', 'off');
text(1.72, 1.012, 'Limit', 'FontName', 'Arial', 'FontSize', 9);

xline(1, ':', 'Color', [0.35, 0.35, 0.35], ...
    'LineWidth', 1.0, 'HandleVisibility', 'off');
if ~isempty(p200_best)
    xline(p200_best.speed_coeff(1), ':', ...
        'Color', [0.55, 0.20, 0.08], ...
        'LineWidth', 1.0, 'HandleVisibility', 'off');
end

xlim([0.72, 1.80]);
ylim([0.70, 1.16]);

text(0.03, 0.92, 'Max normalized joint velocity', ...
    'Units', 'normalized', 'FontName', 'Arial', 'FontSize', 9);
text(0.65, 0.07, 'Normalized forward speed', ...
    'Units', 'normalized', 'FontName', 'Arial', 'FontSize', 9, ...
    'Color', [0.35, 0.35, 0.35]);

if isgraphics(region_handle)
    legend([region_handle, line_handles], ...
        [{'Expanded feasible region'}, legend_labels], ...
        'Location', 'northeast', 'Box', 'on');
else
    legend(line_handles, legend_labels, 'Location', 'northeast', 'Box', 'on');
end

box on;
set(gca, 'FontName', 'Arial', ...
    'FontSize', 9, ...
    'LineWidth', 0.8, ...
    'GridColor', [0.82, 0.82, 0.82], ...
    'GridAlpha', 0.55);

latest_base = 'mvn_baseline_speed_scan_mainstyle_latest';
timestamp_base = ['mvn_baseline_speed_scan_mainstyle_', timestamp];
savefig(fig, [latest_base, '.fig']);
savefig(fig, [timestamp_base, '.fig']);
exportgraphics(fig, [latest_base, '.png'], 'Resolution', 600);
exportgraphics(fig, [latest_base, '.pdf'], 'ContentType', 'vector');
exportgraphics(fig, [timestamp_base, '.png'], 'Resolution', 600);
exportgraphics(fig, [timestamp_base, '.pdf'], 'ContentType', 'vector');
end

function export_paper_quantitative_tables(scan_results, t_sp, y_sp, t_mvn, y_mvn, ...
    dotq_limits, forward_speed_scale, timestamp)
best_rows = scan_results(scan_results.is_best == true, :);
mvn_best = best_rows(best_rows.p_value == 0, :);
sp_best = best_rows(best_rows.p_value == 200, :);

if isempty(mvn_best) || isempty(sp_best)
    warning('Paper table export skipped because best MVN/SP scan rows are missing.');
    return;
end

table_output_dir = 'table_data';
if ~exist(table_output_dir, 'dir')
    mkdir(table_output_dir);
end

mvn_common = select_nearest_speed_coeff(scan_results, 0, 1.0);
sp_common = select_nearest_speed_coeff(scan_results, 200, 1.0);

boundary_table = make_boundary_table(mvn_best, sp_best);
improvement_table = make_improvement_table(mvn_best, sp_best, mvn_common, sp_common, ...
    forward_speed_scale);
[main_table, joint_table, slack_table] = make_main_speed_tables(t_sp, y_sp, ...
    t_mvn, y_mvn, dotq_limits, forward_speed_scale);

write_table_pair(boundary_table, fullfile(table_output_dir, 'paper_table_feasible_boundary'), timestamp);
write_table_pair(improvement_table, fullfile(table_output_dir, 'paper_table_improvement_summary'), timestamp);
write_table_pair(main_table, fullfile(table_output_dir, 'paper_table_main_speed_dynamics'), timestamp);
write_table_pair(joint_table, fullfile(table_output_dir, 'paper_table_joint_detail'), timestamp);
write_table_pair(slack_table, fullfile(table_output_dir, 'paper_table_slack_usage'), timestamp);

save(fullfile(table_output_dir, 'paper_quantitative_tables_latest.mat'), 'boundary_table', 'improvement_table', ...
    'main_table', 'joint_table', 'slack_table', 'scan_results', 'forward_speed_scale');
save(fullfile(table_output_dir, ['paper_quantitative_tables_', timestamp, '.mat']), 'boundary_table', ...
    'improvement_table', 'main_table', 'joint_table', 'slack_table', ...
    'scan_results', 'forward_speed_scale');

write_latex_metric_table(improvement_table, fullfile(table_output_dir, 'paper_table_improvement_summary_latest.tex'), ...
    'Quantitative improvement of SP-VP-RNN over MVN.', ...
    'tab:sp_vp_rnn_improvement');
write_latex_numeric_table(boundary_table, fullfile(table_output_dir, 'paper_table_feasible_boundary_latest.tex'), ...
    'Feasible-region boundary comparison.', 'tab:feasible_boundary');
write_latex_numeric_table(main_table, fullfile(table_output_dir, 'paper_table_main_speed_dynamics_latest.tex'), ...
    'Main-speed time-domain comparison.', 'tab:main_speed_dynamics');
write_paper_table_notes(boundary_table, improvement_table, main_table, slack_table, timestamp, table_output_dir);

fprintf('Exported paper quantitative tables to %s with timestamp %s\n', table_output_dir, timestamp);
end

function row = select_nearest_speed_coeff(results, p_value, target_coeff)
rows = results(results.p_value == p_value, :);
if isempty(rows)
    row = rows;
    return;
end
[~, idx] = min(abs(rows.speed_coeff - target_coeff));
row = rows(idx, :);
end

function T = make_boundary_table(mvn_best, sp_best)
rows = [mvn_best(1, :); sp_best(1, :)];
method = ["MVN"; "SP-VP-RNN"];
feasible_margin_percent = (1 - rows.max_normalized_qdot) * 100;
velocity_margin_rad_s = -rows.max_excess_rad_s;

T = table(method, rows.p_value, rows.speed_coeff, rows.speed_mm_s, ...
    rows.speed_m_s, rows.solver_original_scale, rows.max_normalized_qdot, ...
    feasible_margin_percent, rows.max_excess_rad_s, velocity_margin_rad_s, ...
    rows.critical_joint_id, rows.critical_time_s, rows.critical_abs_qdot_rad_s, ...
    rows.max_abs_qdot_rad_s, rows.min_swing_clearance_mm, ...
    rows.min_clearance_margin_mm, rows.ground_risk_index, ...
    rows.ground_contact_risk_index, rows.ground_contact_safe, ...
    rows.critical_clearance_leg, rows.critical_clearance_time_s, ...
    rows.ode_success, rows.elapsed_s, ...
    'VariableNames', {'method', 'p_value', 'speed_coeff_v_over_vMVN', ...
    'speed_mm_s', 'speed_m_s', 'solver_original_scale', ...
    'max_normalized_qdot', 'qdot_margin_percent', 'max_excess_rad_s', ...
    'velocity_margin_rad_s', 'critical_joint_id', 'critical_time_s', ...
    'critical_abs_qdot_rad_s', 'max_abs_qdot_rad_s', ...
    'min_swing_clearance_mm', 'min_clearance_margin_mm', ...
    'ground_risk_index', 'ground_contact_risk_index', ...
    'ground_contact_safe', 'critical_clearance_leg', ...
    'critical_clearance_time_s', 'ode_success', 'elapsed_s'});
end

function T = make_improvement_table(mvn_best, sp_best, mvn_common, sp_common, ...
    forward_speed_scale)
metric = strings(0, 1);
mvn_value = strings(0, 1);
sp_value = strings(0, 1);
improvement = strings(0, 1);
unit = strings(0, 1);
note = strings(0, 1);

append_metric('Feasible boundary speed coefficient', mvn_best.speed_coeff(1), ...
    sp_best.speed_coeff(1), sp_best.speed_coeff(1) - mvn_best.speed_coeff(1), ...
    'v/v_MVN', 'Right edge of feasible interval.');
append_metric('Feasible boundary speed', mvn_best.speed_mm_s(1), ...
    sp_best.speed_mm_s(1), sp_best.speed_mm_s(1) - mvn_best.speed_mm_s(1), ...
    'mm/s', 'Absolute forward-speed increase.');
append_metric('Feasible boundary speed', mvn_best.speed_m_s(1), ...
    sp_best.speed_m_s(1), sp_best.speed_m_s(1) - mvn_best.speed_m_s(1), ...
    'm/s', 'Same value in SI units.');
append_percent('Feasible-boundary gain', NaN, NaN, ...
    (sp_best.speed_coeff(1) - mvn_best.speed_coeff(1)) / mvn_best.speed_coeff(1) * 100, ...
    '%', 'Relative increase in feasible speed.');
append_metric('Boundary qdot margin', 1 - mvn_best.max_normalized_qdot(1), ...
    1 - sp_best.max_normalized_qdot(1), ...
    (1 - sp_best.max_normalized_qdot(1)) - (1 - mvn_best.max_normalized_qdot(1)), ...
    'normalized', 'Positive value means remaining velocity margin.');
append_metric('Boundary max qdot excess', mvn_best.max_excess_rad_s(1), ...
    sp_best.max_excess_rad_s(1), sp_best.max_excess_rad_s(1) - mvn_best.max_excess_rad_s(1), ...
    'rad/s', 'Negative or near zero means within velocity limit.');
append_metric('Boundary critical joint', mvn_best.critical_joint_id(1), ...
    sp_best.critical_joint_id(1), NaN, 'joint id', ...
    'Joint reaching the largest normalized velocity.');

if ~isempty(mvn_common) && ~isempty(sp_common)
    common_reduction = (mvn_common.max_normalized_qdot(1) - ...
        sp_common.max_normalized_qdot(1)) / mvn_common.max_normalized_qdot(1) * 100;
    append_metric('Max normalized qdot at v/v_MVN=1', ...
        mvn_common.max_normalized_qdot(1), sp_common.max_normalized_qdot(1), ...
        sp_common.max_normalized_qdot(1) - mvn_common.max_normalized_qdot(1), ...
        'ratio', 'Same-speed load comparison.');
    append_percent('Same-speed qdot reduction', NaN, NaN, common_reduction, ...
        '%', 'Reduction of max normalized qdot at v/v_MVN=1.');
    append_metric('Same-speed qdot margin', 1 - mvn_common.max_normalized_qdot(1), ...
        1 - sp_common.max_normalized_qdot(1), ...
        (1 - sp_common.max_normalized_qdot(1)) - ...
        (1 - mvn_common.max_normalized_qdot(1)), ...
        'normalized', 'Additional normalized margin at v/v_MVN=1.');
end

append_metric('Main script speed scale', forward_speed_scale, forward_speed_scale, ...
    0, 'scale', 'Speed multiplier used in main.m time-domain plots.');

T = table(metric, mvn_value, sp_value, improvement, unit, note, ...
    'VariableNames', {'metric', 'MVN', 'SP_VP_RNN', 'improvement', 'unit', 'note'});

    function append_metric(name, mvn, sp, diff_value, unit_value, note_value)
        metric(end + 1, 1) = string(name);
        mvn_value(end + 1, 1) = format_value(mvn);
        sp_value(end + 1, 1) = format_value(sp);
        improvement(end + 1, 1) = format_value(diff_value);
        unit(end + 1, 1) = string(unit_value);
        note(end + 1, 1) = string(note_value);
    end

    function append_percent(name, mvn, sp, diff_value, unit_value, note_value)
        append_metric(name, mvn, sp, diff_value, unit_value, note_value);
    end
end

function [main_table, joint_table, slack_table] = make_main_speed_tables(t_sp, y_sp, ...
    t_mvn, y_mvn, dotq_limits, forward_speed_scale)
discard_time_s = 2;
[sp_main, sp_joint] = compute_method_time_metrics("SP-VP-RNN", t_sp, y_sp, ...
    dotq_limits, discard_time_s, forward_speed_scale);
[mvn_main, mvn_joint] = compute_method_time_metrics("MVN", t_mvn, y_mvn, ...
    dotq_limits, discard_time_s, forward_speed_scale);

main_table = [mvn_main; sp_main];
joint_table = [mvn_joint; sp_joint];
slack_table = compute_slack_usage_table(t_sp, y_sp, discard_time_s, forward_speed_scale);
end

function [method_table, joint_table] = compute_method_time_metrics(method, t, y, ...
    dotq_limits, discard_time_s, forward_speed_scale)
t = t(:);
joint_velocities = [y(:, 11:14), y(:, 16:19)];
valid = t >= discard_time_s - 1e-12;
if ~any(valid)
    valid = true(size(t));
end

valid_t = t(valid);
valid_qdot = joint_velocities(valid, :);
ratio = abs(valid_qdot) ./ dotq_limits;
max_ratio_series = max(ratio, [], 2);
[peak_norm, linear_idx] = max(ratio(:));
[critical_row, critical_col] = ind2sub(size(ratio), linear_idx);
critical_joint_id = critical_col;
critical_time_s = valid_t(critical_row);
critical_abs_qdot_rad_s = abs(valid_qdot(critical_row, critical_col));
critical_limit_rad_s = dotq_limits(critical_col);

solver_end_s = t(end);
solver_completed_10s = solver_end_s >= 10 - 1e-9;

method_table = table(method, forward_speed_scale, solver_end_s, solver_completed_10s, ...
    valid_t(1), valid_t(end), ...
    numel(valid_t), peak_norm, mean(max_ratio_series), ...
    sqrt(mean(max_ratio_series .^ 2)), max(abs(valid_qdot), [], 'all'), ...
    min(dotq_limits - abs(valid_qdot), [], 'all'), ...
    mean(any(ratio > 1, 2)) * 100, critical_joint_id, critical_time_s, ...
    critical_limit_rad_s, critical_abs_qdot_rad_s, ...
    'VariableNames', {'method', 'forward_speed_scale', 'solver_end_s', ...
    'solver_completed_10s', 'analysis_start_s', 'analysis_end_s', ...
    'num_samples', 'peak_normalized_qdot', 'mean_peak_normalized_qdot', ...
    'rms_peak_normalized_qdot', 'max_abs_qdot_rad_s', ...
    'min_velocity_margin_rad_s', 'limit_violation_fraction_percent', ...
    'critical_joint_id', 'critical_time_s', 'critical_limit_rad_s', ...
    'critical_abs_qdot_rad_s'});

joint_id = (1:8)';
limit_rad_s = dotq_limits(:);
peak_abs_qdot_rad_s = zeros(8, 1);
peak_ratio = zeros(8, 1);
rms_qdot_rad_s = zeros(8, 1);
mean_abs_qdot_rad_s = zeros(8, 1);
velocity_margin_rad_s = zeros(8, 1);
time_of_peak_s = zeros(8, 1);

for j = 1:8
    abs_qdot = abs(valid_qdot(:, j));
    [peak_abs_qdot_rad_s(j), idx] = max(abs_qdot);
    peak_ratio(j) = peak_abs_qdot_rad_s(j) / limit_rad_s(j);
    rms_qdot_rad_s(j) = sqrt(mean(valid_qdot(:, j) .^ 2));
    mean_abs_qdot_rad_s(j) = mean(abs_qdot);
    velocity_margin_rad_s(j) = limit_rad_s(j) - peak_abs_qdot_rad_s(j);
    time_of_peak_s(j) = valid_t(idx);
end

joint_table = table(repmat(method, 8, 1), joint_id, limit_rad_s, ...
    peak_abs_qdot_rad_s, peak_ratio, rms_qdot_rad_s, mean_abs_qdot_rad_s, ...
    velocity_margin_rad_s, time_of_peak_s, ...
    'VariableNames', {'method', 'joint_id', 'limit_rad_s', ...
    'peak_abs_qdot_rad_s', 'peak_ratio', 'rms_qdot_rad_s', ...
    'mean_abs_qdot_rad_s', 'velocity_margin_rad_s', 'time_of_peak_s'});
end

function slack_table = compute_slack_usage_table(t, y, discard_time_s, forward_speed_scale)
t = t(:);
valid = t >= discard_time_s - 1e-12;
if ~any(valid)
    valid = true(size(t));
end
valid_t = t(valid);

slack_state = y(valid, [5, 10]);
slack_velocity = y(valid, [15, 20]);
leg = ["Right"; "Left"];
max_abs_delta_mm = max(abs(slack_state), [], 1)';
rms_delta_mm = sqrt(mean(slack_state .^ 2, 1))';
mean_abs_delta_mm = mean(abs(slack_state), 1)';
max_abs_delta_dot_mm_s = max(abs(slack_velocity), [], 1)';
rms_delta_dot_mm_s = sqrt(mean(slack_velocity .^ 2, 1))';
mean_abs_delta_dot_mm_s = mean(abs(slack_velocity), 1)';
active_fraction_percent = mean(abs(slack_velocity) > 1, 1)' * 100;

slack_table = table(repmat("SP-VP-RNN", 2, 1), leg, forward_speed_scale * ones(2, 1), ...
    valid_t(1) * ones(2, 1), valid_t(end) * ones(2, 1), ...
    max_abs_delta_mm, rms_delta_mm, mean_abs_delta_mm, ...
    max_abs_delta_dot_mm_s, rms_delta_dot_mm_s, mean_abs_delta_dot_mm_s, ...
    active_fraction_percent, ...
    'VariableNames', {'method', 'leg', 'forward_speed_scale', ...
    'analysis_start_s', 'analysis_end_s', 'max_abs_delta_mm', ...
    'rms_delta_mm', 'mean_abs_delta_mm', 'max_abs_delta_dot_mm_s', ...
    'rms_delta_dot_mm_s', 'mean_abs_delta_dot_mm_s', ...
    'active_fraction_percent'});
end

function write_table_pair(T, base_name, timestamp)
writetable(T, [base_name, '_latest.csv']);
writetable(T, [base_name, '_', timestamp, '.csv']);
end

function write_latex_numeric_table(T, filename, caption_text, label_text)
fid = fopen(filename, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%% Auto-generated by Copy_of_实验a_直线行走_p200_scan/main.m\n');
fprintf(fid, '\\begin{table}[t]\n\\centering\n');
fprintf(fid, '\\caption{%s}\n', caption_text);
fprintf(fid, '\\label{%s}\n', label_text);
fprintf(fid, '\\small\n');
fprintf(fid, '\\begin{tabular}{%s}\n', repmat('l', 1, width(T)));
fprintf(fid, '\\toprule\n');
fprintf(fid, '%s \\\\\n', strjoin(escape_latex(T.Properties.VariableNames), ' & '));
fprintf(fid, '\\midrule\n');
for i = 1:height(T)
    values = strings(1, width(T));
    for j = 1:width(T)
        values(j) = format_table_cell(T{i, j});
    end
    fprintf(fid, '%s \\\\\n', strjoin(escape_latex(values), ' & '));
end
fprintf(fid, '\\bottomrule\n');
fprintf(fid, '\\end{tabular}\n');
fprintf(fid, '\\end{table}\n');
end

function write_latex_metric_table(T, filename, caption_text, label_text)
fid = fopen(filename, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%% Auto-generated by Copy_of_实验a_直线行走_p200_scan/main.m\n');
fprintf(fid, '\\begin{table}[t]\n\\centering\n');
fprintf(fid, '\\caption{%s}\n', caption_text);
fprintf(fid, '\\label{%s}\n', label_text);
fprintf(fid, '\\small\n');
fprintf(fid, '\\begin{tabular}{lllll}\n');
fprintf(fid, '\\toprule\n');
fprintf(fid, 'Metric & MVN & SP-VP-RNN & Improvement & Unit \\\\\n');
fprintf(fid, '\\midrule\n');
for i = 1:height(T)
    values = [T.metric(i), T.MVN(i), T.SP_VP_RNN(i), T.improvement(i), T.unit(i)];
    fprintf(fid, '%s \\\\\n', strjoin(escape_latex(values), ' & '));
end
fprintf(fid, '\\bottomrule\n');
fprintf(fid, '\\end{tabular}\n');
fprintf(fid, '\\end{table}\n');
end

function write_paper_table_notes(boundary_table, improvement_table, main_table, ...
    slack_table, timestamp, table_output_dir)
fid = fopen(fullfile(table_output_dir, 'paper_table_notes_latest.txt'), 'w');
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, 'Paper quantitative table notes\n');
fprintf(fid, 'Generated timestamp: %s\n\n', timestamp);

fprintf(fid, 'Main files:\n');
fprintf(fid, '- paper_table_improvement_summary_latest.csv/.tex: compact improvement numbers for the main paper.\n');
fprintf(fid, '- paper_table_feasible_boundary_latest.csv/.tex: boundary speed, critical joint, velocity margin, clearance, and contact-risk details.\n');
fprintf(fid, '- paper_table_main_speed_dynamics_latest.csv/.tex: time-domain details at the main.m speed scale.\n');
fprintf(fid, '- paper_table_joint_detail_latest.csv: per-joint peak, RMS, mean absolute velocity, and margin.\n');
fprintf(fid, '- paper_table_slack_usage_latest.csv: SP-VP-RNN slack state and slack-velocity usage.\n');
fprintf(fid, '- paper_quantitative_tables_latest.mat: all tables and source scan data.\n\n');

mvn_boundary = boundary_table(strcmp(boundary_table.method, "MVN"), :);
sp_boundary = boundary_table(strcmp(boundary_table.method, "SP-VP-RNN"), :);
if ~isempty(mvn_boundary) && ~isempty(sp_boundary)
    gain_percent = (sp_boundary.speed_coeff_v_over_vMVN(1) - ...
        mvn_boundary.speed_coeff_v_over_vMVN(1)) / ...
        mvn_boundary.speed_coeff_v_over_vMVN(1) * 100;
    fprintf(fid, 'Key boundary result:\n');
    fprintf(fid, '- MVN feasible boundary: %.6g v/v_MVN, %.6g mm/s.\n', ...
        mvn_boundary.speed_coeff_v_over_vMVN(1), mvn_boundary.speed_mm_s(1));
    fprintf(fid, '- SP-VP-RNN feasible boundary: %.6g v/v_MVN, %.6g mm/s.\n', ...
        sp_boundary.speed_coeff_v_over_vMVN(1), sp_boundary.speed_mm_s(1));
    fprintf(fid, '- Relative feasible-speed gain: %.6g %%.\n\n', gain_percent);
end

same_speed_row = improvement_table(improvement_table.metric == ...
    "Same-speed qdot reduction", :);
if ~isempty(same_speed_row)
    fprintf(fid, 'Same-speed detail:\n');
    fprintf(fid, '- At v/v_MVN = 1, SP-VP-RNN reduces max normalized qdot by %s %s.\n\n', ...
        same_speed_row.improvement(1), same_speed_row.unit(1));
end

mvn_main = main_table(strcmp(main_table.method, "MVN"), :);
sp_main = main_table(strcmp(main_table.method, "SP-VP-RNN"), :);
if ~isempty(mvn_main) && ~isempty(sp_main)
    fprintf(fid, 'Main-speed time-domain detail:\n');
    fprintf(fid, '- MVN peak normalized qdot: %.6g; SP-VP-RNN: %.6g.\n', ...
        mvn_main.peak_normalized_qdot(1), sp_main.peak_normalized_qdot(1));
    fprintf(fid, '- MVN limit-violation fraction: %.6g %%; SP-VP-RNN: %.6g %%.\n', ...
        mvn_main.limit_violation_fraction_percent(1), ...
        sp_main.limit_violation_fraction_percent(1));
    fprintf(fid, '- MVN solver end time: %.6g s; SP-VP-RNN solver end time: %.6g s.\n\n', ...
        mvn_main.solver_end_s(1), sp_main.solver_end_s(1));
end

if ~isempty(slack_table)
    fprintf(fid, 'Slack detail:\n');
    for i = 1:height(slack_table)
        fprintf(fid, '- %s leg: max |delta| = %.6g mm, max |dot delta| = %.6g mm/s, active fraction = %.6g %%.\n', ...
            slack_table.leg(i), slack_table.max_abs_delta_mm(i), ...
            slack_table.max_abs_delta_dot_mm_s(i), ...
            slack_table.active_fraction_percent(i));
    end
end
end

function out = format_table_cell(value)
if iscell(value)
    value = value{1};
end
if isstring(value)
    out = value;
elseif ischar(value)
    out = string(value);
elseif islogical(value)
    out = string(value);
elseif isnumeric(value)
    out = format_value(value);
else
    out = string(value);
end
end

function out = format_value(value)
if isempty(value) || (isnumeric(value) && any(isnan(value(:))))
    out = "n/a";
elseif isnumeric(value)
    if isscalar(value)
        out = string(sprintf('%.6g', value));
    else
        out = string(sprintf('%.6g', value(1)));
    end
elseif islogical(value)
    out = string(value);
else
    out = string(value);
end
end

function out = escape_latex(values)
out = string(values);
for i = 1:numel(out)
    s = char(out(i));
    s = strrep(s, '\', '\textbackslash{}');
    s = strrep(s, '_', '\_');
    s = strrep(s, '%', '\%');
    s = strrep(s, '&', '\&');
    s = strrep(s, '#', '\#');
    out(i) = string(s);
end
end
