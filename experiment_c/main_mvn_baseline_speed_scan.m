%% MVN-baseline speed scan figures
% This script is a compact plotting-only main file for the p=200 experiment.
% It keeps the original main.m plotting style, but only exports:
%   1) mvn_baseline_speed_scan
%   2) all_joint_velocity_with_limits at the SP-VP-RNN boundary speed

clc;
close all;
clear;
format long;

global n m k p sigma gama dotX_max dotX_min q_max q_min forward_speed_scale;
global l1 l2 l3 l4 l5 l6;

cfg = make_default_cfg();
results = load_latest_mvn_baseline_results();
timestamp = datestr(now, 'yyyymmdd_HHMMSS');

plot_mvn_baseline_speed_scan(results, timestamp);

best_sp = results(results.is_best == true & results.p_value == 200, :);
if isempty(best_sp)
    error('No p=200 best row found in the latest MVN-baseline scan results.');
end

forward_speed_scale = best_sp.solver_original_scale(1);
p = 200;
apply_globals(cfg, p, forward_speed_scale);
[t, y] = solve_boundary_case(cfg);
plot_all_joint_velocity(t, y, cfg, best_sp.speed_coeff(1), timestamp);

fprintf('Saved main-style MVN baseline figures with timestamp %s\n', timestamp);

function cfg = make_default_cfg()
cfg = struct();
cfg.n = 10;
cfg.m = 10;
cfg.k = 6;
cfg.num_lambda = 6;
cfg.p = 200;
cfg.sigma = 20;
cfg.gama = 100;
cfg.link_lengths_mm = [90, 0, 10, 370, 310, 100];
cfg.dotq_r_max = [3.4323; 2.7; 1.8; 1.8];
cfg.dotq_r_min = -cfg.dotq_r_max;
cfg.dotq_l_max = [3.4323; 2.7; 1.8; 1.8];
cfg.dotq_l_min = -cfg.dotq_l_max;
cfg.dotq_limits = [cfg.dotq_r_max; cfg.dotq_l_max];
cfg.delta_limit_mm_s = 100;
cfg.qr0 = [0; 0; -pi / 18; pi / 4.3];
cfg.ql0 = [0; 0; -pi / 6.2; pi / 5];
cfg.initial_state = [cfg.qr0; 0; cfg.ql0; 0; zeros(cfg.n + cfg.num_lambda, 1)];
cfg.T = 1;
cfg.num_cycles = 6;
cfg.dt = 0.01;
cfg.ode_rel_tol = 1e-5;
cfg.ode_abs_tol = 1e-8;
cfg.ode_max_step = 0.02;
end

function results = load_latest_mvn_baseline_results()
files = dir('mvn_baseline_speed_scan_results_*.csv');
if isempty(files)
    error('No mvn_baseline_speed_scan_results_*.csv file found.');
end

[~, newest_idx] = max([files.datenum]);
results = readtable(files(newest_idx).name);
results.method_label = string(results.method_label);
results.is_best = logical(results.is_best);
fprintf('Loaded %s\n', files(newest_idx).name);
end

function apply_globals(cfg, p_value, speed_scale)
global n m k p sigma gama dotX_max dotX_min q_max q_min forward_speed_scale;
global l1 l2 l3 l4 l5 l6;

n = cfg.n;
m = cfg.m;
k = cfg.k;
p = p_value;
sigma = cfg.sigma;
gama = cfg.gama;
forward_speed_scale = speed_scale;

links = cfg.link_lengths_mm;
l1 = links(1);
l2 = links(2);
l3 = links(3);
l4 = links(4);
l5 = links(5);
l6 = links(6);
q_max = [];
q_min = [];

dotX_max = [cfg.dotq_r_max; cfg.delta_limit_mm_s; cfg.dotq_l_max; cfg.delta_limit_mm_s];
dotX_min = -dotX_max;
end

function [t, y] = solve_boundary_case(cfg)
tspan = 0:cfg.dt:(cfg.num_cycles * cfg.T);
options = odeset('RelTol', cfg.ode_rel_tol, ...
    'AbsTol', cfg.ode_abs_tol * ones(size(cfg.initial_state)), ...
    'MaxStep', cfg.ode_max_step);

warn_state = warning('off', 'all');
cleanup_warning = onCleanup(@() warning(warn_state));
[t, y] = ode15s(@bepid_VP_RNN, tspan, cfg.initial_state, options);
end

function plot_mvn_baseline_speed_scan(results, timestamp)
fig = figure('Name', 'mvn_baseline_speed_scan', ...
    'Position', [100, 100, 900, 500], 'Color', 'w');
hold on;
grid on;

method_labels = ["p_zero_MVN_baseline", "p_200_slack_penalty"];
legend_labels = {'MVN', 'SP-VP-RNN'};
colors = [0.18, 0.44, 0.62; 0.74, 0.30, 0.18];

best_rows = results(results.is_best == true, :);
p0_best = best_rows(best_rows.p_value == 0, :);
p200_best = best_rows(best_rows.p_value == 200, :);

if ~isempty(p200_best)
    patch([1, p200_best.speed_coeff(1), p200_best.speed_coeff(1), 1], ...
        [0, 0, 1, 1], [0.94, 0.76, 0.48], ...
        'FaceAlpha', 0.24, 'EdgeColor', 'none', ...
        'HandleVisibility', 'off');
end

line_handles = gobjects(1, numel(method_labels));
for idx = 1:numel(method_labels)
    rows = results(results.method_label == method_labels(idx) & results.is_best == false, :);
    [~, order] = sort(rows.speed_coeff);
    rows = rows(order, :);
    line_handles(idx) = plot(rows.speed_coeff, rows.max_normalized_qdot, '-o', ...
        'Color', colors(idx, :), ...
        'MarkerFaceColor', colors(idx, :), ...
        'MarkerSize', 4.2, ...
        'LineWidth', 1.5);
end

yline(1, '--', ...
    'Color', [0.30, 0.30, 0.30], ...
    'LineWidth', 1.2, 'HandleVisibility', 'off');
xline(1, ':', ...
    'Color', [0.10, 0.10, 0.10], ...
    'LineWidth', 1.2, 'HandleVisibility', 'off');
if ~isempty(p200_best)
    xline(p200_best.speed_coeff(1), ':', ...
        'Color', [0.55, 0.20, 0.08], ...
        'LineWidth', 1.2, 'HandleVisibility', 'off');
end

x_right = 0.68;
if ~isempty(p200_best)
    x_right = max(x_right, p200_best.speed_coeff(1) - 1 + 0.06);
end
xlim([1 - x_right, 1 + x_right]);
ylim([0.50, 1.08]);

xlabel('Normalized Forward Speed', 'FontSize', 12);
ylabel('Maximum Normalized Joint Velocity', 'FontSize', 12);
legend(line_handles, legend_labels, ...
    'Location', 'eastoutside', ...
    'FontSize', 10, ...
    'Box', 'on');
box on;

set(gca, 'FontName', 'Arial', ...
    'FontSize', 10, ...
    'LineWidth', 0.9, ...
    'GridColor', [0.82, 0.82, 0.82], ...
    'GridAlpha', 0.55);

base_name = ['mvn_baseline_speed_scan_mainstyle_', timestamp];
savefig(fig, [base_name, '.fig']);
exportgraphics(fig, [base_name, '.png'], 'Resolution', 600);
exportgraphics(fig, [base_name, '.pdf'], 'ContentType', 'vector');
end

function plot_all_joint_velocity(t, y, cfg, speed_coeff, timestamp)
joint_velocities = [y(:, 11:14), y(:, 16:19)];
dotq_r_max = cfg.dotq_r_max;
dotq_r_min = cfg.dotq_r_min;
dotq_l_max = cfg.dotq_l_max;
dotq_l_min = cfg.dotq_l_min;

fig = figure('Name', 'All Joint Velocities', ...
    'Position', [100, 100, 900, 500], 'Color', 'w');
hold on;
grid on;

colors = lines(8);
for i = 1:8
    plot(t, joint_velocities(:, i), ...
        'Color', colors(i, :), ...
        'LineWidth', 1.5);
end

limit_color = [0.30, 0.30, 0.30];
for i = 1:8
    if i <= 4
        qmax = dotq_r_max(i);
        qmin = dotq_r_min(i);
    else
        qmax = dotq_l_max(i - 4);
        qmin = dotq_l_min(i - 4);
    end
    yline(qmax, '--', 'Color', limit_color, 'LineWidth', 0.8, ...
        'HandleVisibility', 'off');
    yline(qmin, '--', 'Color', limit_color, 'LineWidth', 0.8, ...
        'HandleVisibility', 'off');
end

xlabel('Time (s)', 'FontSize', 12);
ylabel('Joint velocity (rad/s)', 'FontSize', 12);
legend({'Joint 1','Joint 2','Joint 3','Joint 4', ...
    'Joint 5','Joint 6','Joint 7','Joint 8'}, ...
    'Location', 'eastoutside', 'FontSize', 10);
xlim([t(1), t(end)]);
ylim([-4, 4]);
box on;
text(0.02, 0.93, sprintf('v / v_{MVN} = %.4f', speed_coeff), ...
    'Units', 'normalized', ...
    'FontName', 'Arial', ...
    'FontSize', 10);

set(gca, 'FontName', 'Arial', ...
    'FontSize', 10, ...
    'LineWidth', 0.9, ...
    'GridColor', [0.82, 0.82, 0.82], ...
    'GridAlpha', 0.55);

base_name = ['all_joint_velocity_with_limits_mainstyle_', timestamp];
savefig(fig, [base_name, '.fig']);
exportgraphics(fig, [base_name, '.png'], 'Resolution', 600);
exportgraphics(fig, [base_name, '.pdf'], 'ContentType', 'vector');
end
