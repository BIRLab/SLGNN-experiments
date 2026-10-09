function result = main(mode)
%MAIN 实验 d：基于实验 a 的直行模型，比较 SLGNN、GNN 和 ZNN。
% 与实验 a、b 一样，直接运行 main 即可完成求解、显示和保存全部结果。
% main('pilot') 只运行第一组初值；main('validate') 只核验解析导数。
% 输入为随本目录提供的共同直行回放；三种网络求解同一个时变问题。
if nargin == 0
    mode = 'full';
end
mode = validatestring(mode, {'full', 'pilot', 'validate', 'api'});
if strcmp(mode, 'api')
    % 兼容原有调参程序；这些句柄和 main 使用同一套实现。
    result = struct('solve', @solve_network, 'measure', @measure, ...
        'residual', @network_residual, 'ppderivative', @ppderivative, ...
        'problem', @network_problem);
    return;
end

%% 定义基本参数（对应实验 a、b 的 main）
clc;
close all;
format long;
root = fileparts(mfilename('fullpath'));
oldPath = path;
oldFolder = pwd;
environmentCleanup = onCleanup(@() restore_environment(oldPath, oldFolder));
addpath(root, '-begin');
cd(root); % 保证同名函数来自实验 d，运行后恢复原工作目录和搜索路径。
for folder = {'results', 'figures', 'logs'}
    if ~isfolder(fullfile(root, folder{1}))
        mkdir(fullfile(root, folder{1}));
    end
end
diary(fullfile(root, 'logs', ['run_' mode '.txt']));
logCleanup = onCleanup(@() diary('off'));
fprintf('Started %s; MATLAB %s\n', char(datetime('now')), version);

%% 系统参数：与实验 a 相同的杆长、变量顺序和速度边界
n = 10;                         % 8 个髋/膝关节速度 + 2 个垂向松弛速度
k = 6;                          % 足端速度等式约束 / 拉格朗日乘子个数
p = 100;
sigma = 20;
gama = 100;
global l1 l2 l3 l4 l5 l6;
l1 = 90; l2 = 0; l3 = 10; l4 = 370; l5 = 310; l6 = 100;
dotq_r_max = [3.4323; 2.7; 1.8; 1.8];
dotq_l_max = dotq_r_max;
delta_r_max = 100;
delta_l_max = 100;
dotX_max = [dotq_r_max; delta_r_max; dotq_l_max; delta_l_max];
dotX_min = -dotX_max;
QP_H = eye(n);
QP_H(5, 5) = 0.2;
QP_H(10, 10) = 0.2;

%% 仿真与求解参数
T = 1;
num = 10;
dt = 0.01;
cfg = struct('p', p, 'sigma', sigma, 'gamma', gama, 'T', num*T, ...
    'dt', dt, 'RelTol', 1e-7, 'AbsTol', 1e-9, 'MaxStep', 0.02, ...
    'seed', 20260909, 'objective_scale', 100, 'velocity_scale', 400, ...
    'threshold', 1e-3, 'repeats', 5);
cfg.H = QP_H;
cfg.limits = dotX_max;
cfg.lower = dotX_min;
cfg.S = diag(dotX_max);
cfg.Hs = cfg.S * QP_H * cfg.S / cfg.objective_scale;
cfg.slack = zeros(k, n);
cfg.slack(3, 5) = 1;
cfg.slack(6, 10) = 1;
methods = {'GNN', 'ZNN', 'SLGNN'};
cfg.methods = methods;

%% 共同直行输入与初值
% 雅可比直接复用实验 a 的 ztm_jacobian；惩罚项复用 PS_Pexp。
% 为比较同一 TVQP，三种网络共享 q(t)、J(t)、期望速度及其解析导数。
inputfile = fullfile(root, 'inputs', 'common_robot_replay.mat');
assert(isfile(inputfile), 'main:MissingInput', ...
    '缺少 inputs/common_robot_replay.mat，请保留随实验 d 提供的共同输入文件。');
load(inputfile, 'input');
assert(input.t(1) <= 0 && input.t(end) >= cfg.T, ...
    'main:InputCoverage', '共同输入未覆盖整个仿真区间。');
P = struct('q', spline(input.t, input.q'), 'c', spline(input.t, input.c'));
P.dq = ppderivative(P.q);
P.dc = ppderivative(P.c);
cfg.t = unique([0:0.001:0.25, 0.26:cfg.dt:cfg.T])';
rng(cfg.seed, 'twister');
initials = zeros(n+k, cfg.repeats);
initials(:, 2:end) = 0.02 * randn(n+k, cfg.repeats-1);
nr = cfg.repeats;
if strcmp(mode, 'pilot')
    nr = 1;
end

%% 解析导数检查和独立参考解
audit = validate_model(P, cfg);
save(fullfile(root, 'results', 'validation.mat'), 'audit', 'cfg');
fprintf('Derivative audit: state %.3g, time %.3g, robot J %.3g\n', ...
    audit.state, audit.time, audit.robot);
if strcmp(mode, 'validate')
    result = struct('audit', audit, 'cfg', cfg);
    return;
end
fprintf('Computing %d independent instantaneous references...\n', numel(cfg.t));
ref = reference_solution(P, cfg);
save(fullfile(root, 'results', 'reference.mat'), 'ref');
assert(max(ref.residual) < 1e-7, 'Reference inaccurate');
assert(max(ref.hard_feasibility) < 1e-6, 'Reference infeasible');
assert(max(ref.hard_kkt) < 1e-8, 'Hard-QP stationarity failed');
fprintf('Reference KKT max %.3g; hard-QP feasibility max %.3g\n', ...
    max(ref.residual), max(ref.hard_feasibility));

%% 求解：相同积分器、初值、参数和输入，只切换网络动力学
rows = struct([]);
runs = cell(nr, numel(methods));
for m = 1:numel(methods)
    cc = cfg;
    cc.T = 0.02;
    solve_network(methods{m}, P, cc, zeros(n+k, 1)); % 不计时热身
end
for trial = 1:nr
    for m = circshift(1:numel(methods), [0 mod(trial-1, numel(methods))])
        fprintf('Trial %d/%d %s...\n', trial, nr, methods{m});
        run = solve_network(methods{m}, P, cfg, initials(:, trial));
        run = measure(run, P, cfg, ref);
        runs{trial, m} = run;
        row = run.metrics;
        row.Trial = trial;
        if isempty(rows), rows = row; else, rows(end+1) = row; end %#ok<AGROW>
        fprintf('  wall %.3fs, RHS %d, tc %.4gs, late residual %.3g, qdot RMSE %.3g\n', ...
            row.Wall_s, row.RHS_calls, row.First_1e3_s, ...
            row.Late_residual_RMS, row.Late_qdot_RMSE_rad_s);
        save(fullfile(root, 'results', 'checkpoint.mat'), ...
            'runs', 'rows', 'cfg', 'audit', 'initials', '-v7.3');
    end
end

%% 结果统计与保存
trialTable = struct2table(rows);
summary = summarize(trialTable, methods);
writetable(trialTable, fullfile(root, 'results', 'all_trials.csv'));
writetable(summary, fullfile(root, 'results', 'summary.csv'));
disp(summary);
save(fullfile(root, 'results', 'comparison.mat'), 'runs', 'summary', ...
    'trialTable', 'cfg', 'audit', 'initials', 'ref', 'mode', '-v7.3');

%% 补充实验：公共增益扫描和积分容差复核
gainTable = table;
toleranceTable = table;
if strcmp(mode, 'full')
    grow = struct([]);
    for gain = [10 30 100 300]
        for m = 1:numel(methods)
            if gain == cfg.gamma
                run = runs{1, m};
            else
                cc = cfg;
                cc.gamma = gain;
                run = measure(solve_network(methods{m}, P, cc, initials(:, 1)), P, cc, ref);
            end
            row = run.metrics;
            row.Gain = gain;
            if isempty(grow), grow = row; else, grow(end+1) = row; end %#ok<AGROW>
            fprintf('Gain %g %s: wall %.3fs late RMS %.3g\n', ...
                gain, methods{m}, row.Wall_s, row.Late_residual_RMS);
        end
    end
    gainTable = struct2table(grow);
    writetable(gainTable, fullfile(root, 'results', 'gain_sensitivity.csv'));
    trow = struct([]);
    for m = 1:numel(methods)
        cc = cfg;
        cc.RelTol = 1e-9;
        cc.AbsTol = 1e-11;
        run = measure(solve_network(methods{m}, P, cc, initials(:, 1)), P, cc, ref);
        if isempty(trow), trow = run.metrics; else, trow(end+1) = run.metrics; end %#ok<AGROW>
        fprintf('Tight tolerance %s: late RMS %.3g\n', methods{m}, run.metrics.Late_residual_RMS);
    end
    toleranceTable = struct2table(trow);
    writetable(toleranceTable, fullfile(root, 'results', 'tolerance_check.csv'));
    disp(gainTable);
    disp(toleranceTable);
    verify_saved_results;
end

%% 绘图：直接显示全部结果，同时保存 PDF、PNG、可编辑 FIG
figures = plot_network_comparison(root, true, strcmp(mode, 'full'));
result = struct('runs', {runs}, 'summary', summary, 'trialTable', trialTable, ...
    'gainTable', gainTable, 'toleranceTable', toleranceTable, 'cfg', cfg, ...
    'audit', audit, 'initials', initials, 'ref', ref, 'figures', figures, 'mode', mode);
if nargout == 0
    assignin('base', 'experiment_d', result);
end
fprintf('Completed %s\nResults: %s\nFigures: %s\n', ...
    char(datetime('now')), fullfile(root, 'results'), fullfile(root, 'figures'));
end

function restore_environment(oldPath, oldFolder)
cd(oldFolder);
path(oldPath);
end

function audit=validate_model(P,cfg)
t=.371;y=[.1*sin((1:10)');.1*cos((1:6)')];[~,D,Ft]=network_residual(t,y,P,cfg);h=1e-6;Dn=zeros(16);
for i=1:16,e=zeros(16,1);e(i)=h;Dn(:,i)=(network_residual(t,y+e,P,cfg)-network_residual(t,y-e,P,cfg))/(2*h);end
Ftn=(network_residual(t+h,y,P,cfg)-network_residual(t-h,y,P,cfg))/(2*h);
audit.state=norm(D-Dn,'fro')/max(1,norm(Dn,'fro'));audit.time=norm(Ft-Ftn)/max(1,norm(Ftn));
q=ppval(P.q,t);dq=ppval(P.dq,t);[J,dJ]=ztm_jacobian(q,dq);
Jp=ztm_jacobian(q+h*dq,dq);Jm=ztm_jacobian(q-h*dq,dq);
audit.robot=norm(dJ-(Jp-Jm)/(2*h),'fro')/max(1,norm(dJ,'fro'));
assert(audit.state<1e-6 && audit.time<1e-5 && audit.robot<1e-5,'Derivative check failed');
audit.rank=rank(J);audit.date=char(datetime('now'));
% Test near an active joint bound where exponential Hessian is material.
y(4)=.97;[F,D,~,third]=network_residual(t,y,P,cfg);Dn=zeros(16);Gn=zeros(16);
correction=zeros(16);correction(1:10,1:10)=diag(third.*F(1:10));Gexact=-cfg.gamma*(D'*D+correction);
for i=1:16
 e=zeros(16,1);e(i)=h;[fp,dp]=network_residual(t,y+e,P,cfg);[fm,dm]=network_residual(t,y-e,P,cfg);
 Dn(:,i)=(fp-fm)/(2*h);Gn(:,i)=-cfg.gamma*(dp'*fp-dm'*fm)/(2*h);
end
audit.active_state=norm(D-Dn,'fro')/norm(D,'fro');audit.GNN_jacobian=norm(Gexact-Gn,'fro')/norm(Gexact,'fro');
assert(audit.active_state<1e-6 && audit.GNN_jacobian<1e-6,'Active-penalty derivative check failed');
end

function ref=reference_solution(P,cfg)
nt=numel(cfg.t);ref.y=zeros(nt,16);ref.hard=zeros(nt,10);ref.residual=zeros(nt,1);ref.hard_feasibility=zeros(nt,1);
y=zeros(16,1);
for i=1:nt
 t=cfg.t(i);[A,~,c]=network_problem(t,P,cfg);
 for it=1:80
  [F,D]=network_residual(t,y,P,cfg);nr=norm(F);if nr<1e-11,break;end
  dy=-D\F;alpha=1;
  for ls=1:45
   yn=y+alpha*dy;Fn=network_residual(t,yn,P,cfg);
   if norm(Fn)<(1-1e-4*alpha)*nr,break;end
   alpha=alpha*.5;
  end
  y=yn;
 end
 ref.y(i,:)=y';ref.residual(i)=norm(network_residual(t,y,P,cfg));
 [zh,kkt]=box_qp(cfg.Hs,A,c,y(1:10));ref.hard(i,:)=(cfg.S*zh)';
 ref.hard_feasibility(i)=max([norm(A*zh-c,Inf);max(abs(zh)-1);0]);ref.hard_kkt(i)=kkt;
 if mod(i,250)==0,fprintf('  Reference %d/%d\n',i,nt);end
end
ref.x=ref.y(:,1:10)*cfg.S;
ref.penalty_bias_qdot_RMSE=sqrt(mean((ref.x(:,[1:4 6:9])-ref.hard(:,[1:4 6:9])).^2,'all'));
end

function [z,kkt]=box_qp(H,A,c,z)
% Feasible primal active set. Penalty optimum supplies strictly feasible seed.
assert(max(abs(z))<=1+1e-8 && norm(A*z-c)<1e-7,'Hard-QP seed infeasible');
G=[eye(10);-eye(10)];active=[];
for it=1:100
 B=[A;G(active,:)];b=[c;ones(numel(active),1)];
 w=[H B';B zeros(size(B,1))]\[zeros(10,1);b];
 delta=w(1:10)-z;
 if norm(delta)<1e-10
  lm=w(17:end);
  if isempty(lm) || min(lm)>=-1e-9
   z=w(1:10);kkt=norm(H*z+B'*w(11:end),Inf);return;
  end
  [~,j]=min(lm);active(j)=[];
 else
  gd=G*delta;slack=1-G*z;valid=find(gd>1e-12);valid=setdiff(valid,active);
  alpha=1;block=[];
  if ~isempty(valid),[alpha0,j]=min(slack(valid)./gd(valid));if alpha0<1,alpha=max(0,alpha0);block=valid(j);end,end
  z=z+alpha*delta;
  if ~isempty(block),active=[active block];end %#ok<AGROW>
 end
end
error('Hard-QP active set failed to converge');
end

function run=measure(run,P,cfg,ref)
t=run.t;nt=numel(t);err=zeros(nt,1);task=zeros(nt,1);
for i=1:nt,F=network_residual(t(i),run.y(i,:)',P,cfg);err(i)=norm(F);task(i)=norm(F(11:16))*cfg.velocity_scale;end
x=run.y(:,1:10)*cfg.S;qi=[1:4 6:9];qerror=sqrt(mean((x(:,qi)-ref.x(:,qi)).^2,2));
normerror=vecnorm((x-ref.x)./cfg.limits',2,2);harderror=sqrt(mean((x(:,qi)-ref.hard(:,qi)).^2,2));late=t>=1;ss=t>=5;
run.residual=err;run.task=task;run.qerror=qerror;run.normerror=normerror;run.x=x;
row.Method=string(run.method);row.Wall_s=run.wall;row.RHS_calls=run.calls;row.Accepted_steps=run.steps;
first=find(err<=cfg.threshold,1);row.First_1e3_s=NaN;if ~isempty(first),row.First_1e3_s=t(first);end
lastbad=find(err>cfg.threshold,1,'last');row.Sustained_1e3_s=NaN;
if isempty(lastbad),row.Sustained_1e3_s=0;elseif lastbad<nt,row.Sustained_1e3_s=t(lastbad+1);end
row.Residual_time_mean=trapz(t,err)/(t(end)-t(1));
row.Late_residual_RMS=sqrt(trapz(t(late),err(late).^2)/(t(end)-1));
row.Steady_residual_RMS=sqrt(trapz(t(ss),err(ss).^2)/(t(end)-5));
row.Late_qdot_RMSE_rad_s=sqrt(trapz(t(late),qerror(late).^2)/(t(end)-1));
row.Late_hardQP_qdot_RMSE_rad_s=sqrt(trapz(t(late),harderror(late).^2)/(t(end)-1));
row.Late_task_RMS_mm_s=sqrt(trapz(t(late),task(late).^2)/(t(end)-1));
row.Max_normalized_bound_violation=max([0;max(abs(x)./cfg.limits'-1,[],2)]);
row.Final_residual=err(end);run.metrics=row;
end

function summary=summarize(T,methods)
rows=struct([]);
for m=1:3
 U=T(T.Method==methods{m},:);row.Method=string(methods{m});row.N=height(U);
 row.Wall_median_s=median(U.Wall_s);row.Wall_min_s=min(U.Wall_s);row.Wall_max_s=max(U.Wall_s);
 row.RHS_median=median(U.RHS_calls);row.First_1e3_median_s=median(U.First_1e3_s);row.Sustained_1e3_median_s=median(U.Sustained_1e3_s);
 row.Late_residual_RMS_median=median(U.Late_residual_RMS);row.Late_qdot_RMSE_median_rad_s=median(U.Late_qdot_RMSE_rad_s);
 row.Late_hardQP_qdot_RMSE_median_rad_s=median(U.Late_hardQP_qdot_RMSE_rad_s);row.Late_task_RMS_median_mm_s=median(U.Late_task_RMS_mm_s);
 if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
end
summary=struct2table(rows);
end
