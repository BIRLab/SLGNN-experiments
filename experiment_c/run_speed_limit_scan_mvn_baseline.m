%% Speed limit scan normalized by the MVN critical speed
% The horizontal speed coefficient is v / v_MVN, where v_MVN is the strict
% p = 0 maximum feasible speed from the previous scan.

clc;
clear;
close all;
format long;

global n m k p sigma gama dotX_max dotX_min q_max q_min forward_speed_scale;
global l1 l2 l3 l4 l5 l6;

cfg = make_scan_cfg();
results = table();
cases = struct('p_value', {0, 200}, ...
    'label', {'p_zero_MVN_baseline', 'p_200_slack_penalty'});

fprintf('MVN-baseline speed scan started.\n');
fprintf('Baseline v_MVN = %.9f mm/s (original solver scale %.12f).\n', ...
    cfg.baseline_speed_mm_s, cfg.baseline_original_scale);
fprintf('Reported speed coefficient is v / v_MVN.\n');

for case_idx = 1:numel(cases)
    p_case = cases(case_idx).p_value;
    fprintf('\nCase %s (p = %.6g)\n', cases(case_idx).label, p_case);

    low = 0;
    high = cfg.initial_high_coeff;
    case_rows = table();

    while true
        row = solve_one_coeff(high, p_case, cfg);
        case_rows = [case_rows; row]; %#ok<AGROW>
        fprintf('  coeff %.5f, speed %.2f mm/s: max ratio %.9f, excess %.6g rad/s, feasible %d\n', ...
            row.speed_coeff, row.speed_mm_s, row.max_normalized_qdot, ...
            row.max_excess_rad_s, row.feasible_strict);

        if ~row.feasible_strict || high >= cfg.max_coeff
            break;
        end
        low = high;
        high = min(high + cfg.expand_step_coeff, cfg.max_coeff);
    end

    if high >= cfg.max_coeff && case_rows.feasible_strict(end)
        best_coeff = high;
    else
        for iter = 1:cfg.binary_iterations
            mid = 0.5 * (low + high);
            row = solve_one_coeff(mid, p_case, cfg);
            case_rows = [case_rows; row]; %#ok<AGROW>
            fprintf('  refine %.5f, speed %.2f mm/s: max ratio %.9f, excess %.6g rad/s, feasible %d\n', ...
                row.speed_coeff, row.speed_mm_s, row.max_normalized_qdot, ...
                row.max_excess_rad_s, row.feasible_strict);
            if row.feasible_strict
                low = mid;
            else
                high = mid;
            end
        end
        best_coeff = low;
    end

    best_row = solve_one_coeff(best_coeff, p_case, cfg);
    best_row.is_best = true;
    case_rows.is_best(:) = false;
    case_rows = [case_rows; best_row]; %#ok<AGROW>
    case_rows.method_label = repmat(string(cases(case_idx).label), height(case_rows), 1);
    results = [results; case_rows]; %#ok<AGROW>

    fprintf('  BEST: coeff %.5f, speed %.2f mm/s (%.5f m/s), max ratio %.9f\n', ...
        best_row.speed_coeff, best_row.speed_mm_s, best_row.speed_m_s, ...
        best_row.max_normalized_qdot);
end

results = movevars(results, 'method_label', 'Before', 1);
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
csv_file = ['mvn_baseline_speed_scan_results_', timestamp, '.csv'];
mat_file = ['mvn_baseline_speed_scan_results_', timestamp, '.mat'];
writetable(results, csv_file);
save(mat_file, 'results', 'cfg');

summary = results(results.is_best, :);
disp(summary(:, {'method_label', 'p_value', 'speed_coeff', ...
    'speed_mm_s', 'speed_m_s', 'solver_original_scale', ...
    'max_normalized_qdot', 'max_excess_rad_s', 'critical_joint_id', ...
    'min_swing_clearance_mm', 'min_clearance_margin_mm', ...
    'ground_contact_risk_index', 'ground_contact_safe'}));

if height(summary) == 2
    p200_row = summary(summary.p_value == 200, :);
    p0_row = summary(summary.p_value == 0, :);
    if height(p200_row) == 1 && height(p0_row) == 1
        improvement_coeff = p200_row.speed_coeff - p0_row.speed_coeff;
        improvement_speed_mm_s = p200_row.speed_mm_s - p0_row.speed_mm_s;
        improvement_percent = improvement_coeff / p0_row.speed_coeff * 100;
        fprintf('\nNormalized feasible-domain improvement from p=0 to p=200:\n');
        fprintf('  coefficient interval: [%.5f, %.5f], width %.5f (%.2f%% over MVN)\n', ...
            p0_row.speed_coeff, p200_row.speed_coeff, improvement_coeff, ...
            improvement_percent);
        fprintf('  speed interval: [%.2f, %.2f] mm/s, width %.2f mm/s (%.5f m/s)\n', ...
            p0_row.speed_mm_s, p200_row.speed_mm_s, improvement_speed_mm_s, ...
            improvement_speed_mm_s / 1000);
    end
end

plot_scan_results(results, timestamp);
plot_clearance_results(results, timestamp);
plot_contact_risk_results(results, timestamp);
fprintf('\nSaved %s and %s\n', csv_file, mat_file);

function cfg = make_scan_cfg()
cfg = struct();
cfg.n = 10;
cfg.m = 10;
cfg.k = 6;
cfg.num_lambda = 6;
cfg.sigma = 20;
cfg.gama = 100;
cfg.link_lengths_mm = [90, 0, 10, 370, 310, 100];
cfg.dotq_r_max = [3.4323; 2.7; 1.8; 1.8];
cfg.dotq_l_max = [3.4323; 2.7; 1.8; 1.8];
cfg.dotq_limits = [cfg.dotq_r_max; cfg.dotq_l_max];
cfg.delta_limit_mm_s = 100;
cfg.qr0 = [0; 0; -pi / 18; pi / 4.3];
cfg.ql0 = [0; 0; -pi / 6.2; pi / 5];
cfg.initial_state = [cfg.qr0; 0; cfg.ql0; 0; zeros(cfg.n + cfg.num_lambda, 1)];
cfg.T = 1;
cfg.num_cycles = 6;
cfg.dt = 0.01;
cfg.discard_time_s = 2;
cfg.ode_rel_tol = 1e-5;
cfg.ode_abs_tol = 1e-8;
cfg.ode_max_step = 0.02;

cfg.original_nominal_body_speed_mm_s = 200;
cfg.baseline_original_scale = 0.6884765625;
cfg.baseline_speed_mm_s = cfg.original_nominal_body_speed_mm_s * cfg.baseline_original_scale;

cfg.initial_high_coeff = 0.6;
cfg.expand_step_coeff = 0.05;
cfg.max_coeff = 2.5;
cfg.binary_iterations = 10;
cfg.joint_ids = [1, 2, 3, 4, 5, 6, 7, 8];
cfg.clearance_required_mm = 20;
cfg.clearance_phase_window = [0.05, 0.95];
cfg.ground_contact_tolerance_mm = 0.5;
end

function row = solve_one_coeff(speed_coeff, p_value, cfg)
solver_original_scale = speed_coeff * cfg.baseline_original_scale;
row = solve_one_original_scale(solver_original_scale, p_value, cfg);
row.speed_coeff = speed_coeff;
row.speed_mm_s = cfg.baseline_speed_mm_s * speed_coeff;
row.speed_m_s = row.speed_mm_s / 1000;
row = movevars(row, 'speed_coeff', 'Before', 'solver_original_scale');
end

function row = solve_one_original_scale(solver_original_scale, p_value, cfg)
global n m k p sigma gama dotX_max dotX_min q_max q_min forward_speed_scale;
global l1 l2 l3 l4 l5 l6;

n = cfg.n;
m = cfg.m;
k = cfg.k;
p = p_value;
sigma = cfg.sigma;
gama = cfg.gama;
links = cfg.link_lengths_mm;
l1 = links(1);
l2 = links(2);
l3 = links(3);
l4 = links(4);
l5 = links(5);
l6 = links(6);
q_max = [];
q_min = [];
forward_speed_scale = solver_original_scale;

dotX_max = [cfg.dotq_r_max; cfg.delta_limit_mm_s; cfg.dotq_l_max; cfg.delta_limit_mm_s];
dotX_min = -dotX_max;

tspan = 0:cfg.dt:(cfg.num_cycles * cfg.T);
options = odeset('RelTol', cfg.ode_rel_tol, ...
    'AbsTol', cfg.ode_abs_tol * ones(size(cfg.initial_state)), ...
    'MaxStep', cfg.ode_max_step);

warn_state = warning('off', 'all');
cleanup_warning = onCleanup(@() warning(warn_state));

elapsed_s = NaN;
ode_success = false;
failure_reason = "none";
max_ratio = NaN;
critical_joint_id = NaN;
critical_time_s = NaN;
critical_limit_rad_s = NaN;
critical_abs_qdot_rad_s = NaN;
max_abs_qdot = NaN;
max_excess = NaN;
min_swing_clearance_mm = NaN;
min_clearance_margin_mm = NaN;
ground_risk_index = NaN;
ground_contact_risk_index = NaN;
ground_contact_safe = false;
critical_clearance_leg = "";
critical_clearance_time_s = NaN;

try
    timer_start = tic;
    [t, y] = ode15s(@bepid_VP_RNN, tspan, cfg.initial_state, options);
    elapsed_s = toc(timer_start);
    ode_success = numel(t) == numel(tspan);
    if ~ode_success
        failure_reason = "ode_early_termination";
    end

    valid_idx = t >= cfg.discard_time_s - 1e-12;
    valid_t = t(valid_idx);
    joint_velocities = [y(:, 11:14), y(:, 16:19)];
    valid_abs_qdot = abs(joint_velocities(valid_idx, :));
    ratios = valid_abs_qdot ./ cfg.dotq_limits';
    [max_ratio, linear_idx] = max(ratios(:));
    [critical_row, critical_col] = ind2sub(size(ratios), linear_idx);
    critical_joint_id = cfg.joint_ids(critical_col);
    critical_time_s = valid_t(critical_row);
    critical_limit_rad_s = cfg.dotq_limits(critical_col);
    critical_abs_qdot_rad_s = valid_abs_qdot(critical_row, critical_col);
    max_abs_qdot = max(valid_abs_qdot, [], 'all');
    excess_by_joint = valid_abs_qdot - cfg.dotq_limits';
    max_excess = max(excess_by_joint, [], 'all');

    clearance = compute_swing_clearance_metrics(t, y, solver_original_scale, cfg);
    min_swing_clearance_mm = clearance.min_swing_clearance_mm;
    min_clearance_margin_mm = clearance.min_clearance_margin_mm;
    ground_risk_index = clearance.ground_risk_index;
    ground_contact_risk_index = clearance.ground_contact_risk_index;
    ground_contact_safe = clearance.ground_contact_safe;
    critical_clearance_leg = clearance.critical_leg;
    critical_clearance_time_s = clearance.critical_time_s;
catch ME
    failure_reason = string(['ode_error: ', ME.identifier]);
end

feasible_strict = ode_success && isfinite(max_ratio) && max_ratio <= 1;
if ~feasible_strict && failure_reason == "none"
    failure_reason = "qdot_limit";
end

row = table(string(""), p_value, solver_original_scale, NaN, NaN, ...
    max_ratio, max_excess, feasible_strict, critical_joint_id, ...
    critical_time_s, critical_limit_rad_s, critical_abs_qdot_rad_s, ...
    max_abs_qdot, min_swing_clearance_mm, min_clearance_margin_mm, ...
    ground_risk_index, ground_contact_risk_index, ground_contact_safe, ...
    critical_clearance_leg, critical_clearance_time_s, ode_success, ...
    failure_reason, elapsed_s, false, ...
    'VariableNames', {'method_label', 'p_value', 'solver_original_scale', ...
    'speed_mm_s', 'speed_m_s', 'max_normalized_qdot', ...
    'max_excess_rad_s', 'feasible_strict', 'critical_joint_id', ...
    'critical_time_s', 'critical_limit_rad_s', 'critical_abs_qdot_rad_s', ...
    'max_abs_qdot_rad_s', 'min_swing_clearance_mm', ...
    'min_clearance_margin_mm', 'ground_risk_index', ...
    'ground_contact_risk_index', 'ground_contact_safe', ...
    'critical_clearance_leg', 'critical_clearance_time_s', ...
    'ode_success', 'failure_reason', 'elapsed_s', 'is_best'});
end

function clearance = compute_swing_clearance_metrics(t, y, solver_original_scale, cfg)
t = t(:);
traj = reconstruct_foot_trajectories_local(t, y, cfg);
period_s = 2 / solver_original_scale;
half_period_s = period_s / 2;
phase_window = cfg.clearance_phase_window;

min_clearance = inf;
min_margin = inf;
critical_leg = "";
critical_time = NaN;

period_start = 0;
while period_start < t(end) - 1e-12
    swings = { ...
        'R', period_start, period_start + half_period_s; ...
        'L', period_start + half_period_s, period_start + period_s};

    for swing_idx = 1:size(swings, 1)
        leg = swings{swing_idx, 1};
        t0 = swings{swing_idx, 2};
        t1 = swings{swing_idx, 3};
        if t0 < cfg.discard_time_s - 1e-12
            continue;
        end

        idx = find(t >= t0 - 1e-12 & t <= t1 + 1e-12);
        if numel(idx) < 3
            continue;
        end

        p_phase = (t(idx) - t0) / (t1 - t0);
        p_phase = min(max(p_phase, 0), 1);
        interior = p_phase >= phase_window(1) & p_phase <= phase_window(2);
        if ~any(interior)
            continue;
        end

        if leg == 'R'
            foot = traj.right(idx, :);
        else
            foot = traj.left(idx, :);
        end

        z_start = foot(1, 3);
        z_end = foot(end, 3);
        z_base = (1 - p_phase) .* z_start + p_phase .* z_end;
        c_raw = foot(:, 3) - z_base;

        mid_mask = p_phase >= 0.4 & p_phase <= 0.6;
        mid_lift = median(c_raw(mid_mask), 'omitnan');
        z_sign = 1;
        if isfinite(mid_lift) && mid_lift < 0
            z_sign = -1;
        end

        c_actual = z_sign * c_raw;
        c_safe = cfg.clearance_required_mm * sin(pi * p_phase);
        margin = c_actual - c_safe;

        interior_idx = find(interior);
        [segment_min_margin, local_idx] = min(margin(interior));
        row_idx = interior_idx(local_idx);
        segment_min_clearance = min(c_actual(interior));

        if segment_min_margin < min_margin
            min_margin = segment_min_margin;
            min_clearance = segment_min_clearance;
            critical_leg = string(leg);
            critical_time = t(idx(row_idx));
        end
    end

    period_start = period_start + period_s;
end

if isinf(min_margin)
    min_margin = NaN;
    min_clearance = NaN;
end

clearance = struct();
clearance.min_swing_clearance_mm = min_clearance;
clearance.min_clearance_margin_mm = min_margin;
clearance.ground_risk_index = max(0, -min_margin) / cfg.clearance_required_mm;
clearance.ground_contact_risk_index = max(0, -min_clearance) / cfg.clearance_required_mm;
clearance.ground_contact_safe = min_clearance >= -cfg.ground_contact_tolerance_mm;
clearance.critical_leg = critical_leg;
clearance.critical_time_s = critical_time;
end

function traj = reconstruct_foot_trajectories_local(t, y, cfg)
nT = numel(t);
links = cfg.link_lengths_mm;
right = nan(nT, 3);
left = nan(nT, 3);

for i = 1:nT
    qr = [y(i, 1:4)'; -y(i, 3) - y(i, 4)];
    ql = [y(i, 6:9)'; -y(i, 8) - y(i, 9)];
    q = [0; qr; ql];

    [xr, yr, zr, xl, yl, zl] = ztmkinematics( ...
        links(1), links(2), links(3), links(4), links(5), links(6), q);

    right(i, :) = [xr(end - 1), yr(end - 1), zr(end - 1)];
    left(i, :) = [xl(end - 1), yl(end - 1), zl(end - 1)];
end

traj = struct();
traj.right = right;
traj.left = left;
end

function plot_scan_results(results, timestamp)
fig = figure('Name', 'MVN-baseline speed scan', 'Color', 'w', ...
    'Visible', 'off', 'Position', [100, 100, 900, 500]);
hold on;
method_labels = ["p_zero_MVN_baseline", "p_200_slack_penalty"];
legend_labels = {'MVN', 'SP-VP-RNN'};
colors = [0.15, 0.40, 0.70; 0.75, 0.25, 0.15];

best_rows = results(results.is_best == true, :);
p0_best = best_rows(best_rows.p_value == 0, :);
p200_best = best_rows(best_rows.p_value == 200, :);
if ~isempty(p200_best)
    patch([1, p200_best.speed_coeff(1), p200_best.speed_coeff(1), 1], ...
        [0, 0, 1, 1], [0.94, 0.76, 0.48], ...
        'FaceAlpha', 0.22, 'EdgeColor', 'none', ...
        'HandleVisibility', 'off');
end

line_handles = gobjects(1, numel(method_labels));
for idx = 1:numel(method_labels)
    rows = results(results.method_label == method_labels(idx) & ~results.is_best, :);
    [~, order] = sort(rows.speed_coeff);
    rows = rows(order, :);
    line_handles(idx) = plot(rows.speed_coeff, rows.max_normalized_qdot, '-o', ...
        'Color', colors(idx, :), 'MarkerFaceColor', colors(idx, :), ...
        'MarkerSize', 4.2, 'LineWidth', 1.5);
end
yline(1, '--', 'Color', [0.30, 0.30, 0.30], ...
    'LineWidth', 1.2, 'HandleVisibility', 'off');
xline(1, ':', 'Color', [0.10, 0.10, 0.10], ...
    'LineWidth', 1.2, 'HandleVisibility', 'off');
if ~isempty(p200_best)
    xline(p200_best.speed_coeff(1), ':', ...
        'Color', [0.55, 0.20, 0.08], 'LineWidth', 1.15, ...
        'HandleVisibility', 'off');
end

x_right = 0.68;
if ~isempty(p200_best)
    x_right = max(x_right, p200_best.speed_coeff(1) - 1 + 0.06);
end
xlim([1 - x_right, 1 + x_right]);
ylim([0.50, 1.08]);

grid on;
box on;
xlabel('Normalized Forward Speed', 'FontSize', 12);
ylabel('Maximum Normalized Joint Velocity', 'FontSize', 12);
legend(line_handles, legend_labels, ...
    'Location', 'eastoutside', ...
    'FontSize', 10, ...
    'Box', 'on');
set(gca, 'FontName', 'Arial', ...
    'FontSize', 10, ...
    'LineWidth', 0.9, ...
    'GridColor', [0.82, 0.82, 0.82], ...
    'GridAlpha', 0.55);

png_file = ['mvn_baseline_speed_scan_', timestamp, '.png'];
pdf_file = ['mvn_baseline_speed_scan_', timestamp, '.pdf'];
fig_file = ['mvn_baseline_speed_scan_', timestamp, '.fig'];
savefig(fig, fig_file);
exportgraphics(fig, png_file, 'Resolution', 300);
exportgraphics(fig, pdf_file, 'ContentType', 'vector');
end

function plot_clearance_results(results, timestamp)
fig = figure('Name', 'Swing clearance safety scan', 'Color', 'w', ...
    'Visible', 'off', 'Position', [100, 100, 760, 430]);
hold on;
labels = unique(results.method_label, 'stable');
colors = [0.15, 0.40, 0.70; 0.75, 0.25, 0.15];
for idx = 1:numel(labels)
    rows = results(results.method_label == labels(idx) & ~results.is_best, :);
    [~, order] = sort(rows.speed_coeff);
    rows = rows(order, :);
    plot(rows.speed_coeff, rows.min_clearance_margin_mm, '-o', ...
        'Color', colors(idx, :), 'MarkerFaceColor', colors(idx, :), ...
        'LineWidth', 1.4, 'DisplayName', char(labels(idx)));
end
yline(0, '--k', 'Safety boundary', 'LineWidth', 1.1, 'HandleVisibility', 'off');
xline(1, ':', 'MVN boundary', 'LineWidth', 1.1, 'HandleVisibility', 'off');
grid on;
box on;
xlabel('Normalized forward speed, v / v_{MVN}');
ylabel('Minimum swing clearance margin (mm)');
title('Swing-foot ground-clearance safety margin');
legend('Location', 'southwest', 'Interpreter', 'none');
set(gca, 'FontName', 'Arial', 'FontSize', 10, 'LineWidth', 0.9);
png_file = ['mvn_baseline_clearance_margin_', timestamp, '.png'];
pdf_file = ['mvn_baseline_clearance_margin_', timestamp, '.pdf'];
exportgraphics(fig, png_file, 'Resolution', 300);
exportgraphics(fig, pdf_file, 'ContentType', 'vector');
end

function plot_contact_risk_results(results, timestamp)
fig = figure('Name', 'Swing ground-contact risk scan', 'Color', 'w', ...
    'Visible', 'off', 'Position', [100, 100, 760, 430]);
hold on;
labels = unique(results.method_label, 'stable');
colors = [0.15, 0.40, 0.70; 0.75, 0.25, 0.15];
for idx = 1:numel(labels)
    rows = results(results.method_label == labels(idx) & ~results.is_best, :);
    [~, order] = sort(rows.speed_coeff);
    rows = rows(order, :);
    plot(rows.speed_coeff, rows.ground_contact_risk_index, '-o', ...
        'Color', colors(idx, :), 'MarkerFaceColor', colors(idx, :), ...
        'LineWidth', 1.4, 'DisplayName', char(labels(idx)));
end
yline(0, '--k', 'No contact risk', 'LineWidth', 1.1, 'HandleVisibility', 'off');
xline(1, ':', 'MVN boundary', 'LineWidth', 1.1, 'HandleVisibility', 'off');
grid on;
box on;
xlabel('Normalized forward speed, v / v_{MVN}');
ylabel('Ground-contact risk index');
title('Swing-foot ground-contact risk');
legend('Location', 'northwest', 'Interpreter', 'none');
set(gca, 'FontName', 'Arial', 'FontSize', 10, 'LineWidth', 0.9);
png_file = ['mvn_baseline_ground_contact_risk_', timestamp, '.png'];
pdf_file = ['mvn_baseline_ground_contact_risk_', timestamp, '.pdf'];
exportgraphics(fig, png_file, 'Resolution', 300);
exportgraphics(fig, pdf_file, 'ContentType', 'vector');
end
