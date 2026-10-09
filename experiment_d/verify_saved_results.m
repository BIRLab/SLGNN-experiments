function verify_saved_results
% Checks the saved experiments independently of plotting/report formatting.
root=fileparts(mfilename('fullpath'));d=load(fullfile(root,'results','comparison.mat'));
assert(max(d.ref.residual)<1e-7);assert(max(d.ref.hard_feasibility)<1e-6);
assert(max(d.ref.hard_kkt)<1e-8);
assert(d.audit.active_state<1e-6 && d.audit.GNN_jacobian<1e-6);
assert(size(d.initials,2)==5);
metrics=readtable(fullfile(root,'results','summary.csv'),'TextType','string');
names={'GNN','ZNN','smg-RNN'};if isfield(d.cfg,'methods'),names=d.cfg.methods;end;max_roundtrip=0;max_law_deviation=0;
for m=1:3
 errors=zeros(5,1);clocks=zeros(5,1);
 for j=1:5
  r=d.runs{j,m};t=r.t;assert(t(end)==10 && all(isfinite(r.y),'all'));
  assert(norm(r.y(1,:)'-d.initials(:,j))<1e-12);
  qi=[1:4 6:9];late=t>=1;
  error2=mean((r.x(:,qi)-d.ref.x(:,qi)).^2,2);
  errors(j)=sqrt(trapz(t(late),error2(late))/9);clocks(j)=r.wall;
  max_roundtrip=max(max_roundtrip,abs(errors(j)-r.metrics.Late_qdot_RMSE_rad_s));
  if m>1
   early=t<=.08;
   if m==2,ideal=r.residual(1)*exp(-d.cfg.gamma*t(early));
   else,ideal=r.residual(1)*exp(-d.cfg.gamma*expm1(t(early)));end
   dev=max(abs(r.residual(early)-ideal))/max(1,r.residual(1));
   max_law_deviation=max(max_law_deviation,dev);
  end
 end
 row=metrics(metrics.Method==names{m},:);
 assert(abs(median(errors)-row.Late_qdot_RMSE_median_rad_s)<1e-12);
 assert(abs(median(clocks)-row.Wall_median_s)<1e-10);
end
assert(max_roundtrip<1e-12 && max_law_deviation<1e-4);
G=readtable(fullfile(root,'results','gain_sensitivity.csv'));assert(height(G)==12);
T=readtable(fullfile(root,'results','tolerance_check.csv'));assert(height(T)==3);
report=struct('passed',true,'paired_trials',15,'gain_rows',height(G),'tolerance_rows',height(T),...
 'reference_KKT_max',max(d.ref.residual),'hard_QP_KKT_max',max(d.ref.hard_kkt),...
 'metric_roundtrip_max',max_roundtrip,'initial_decay_law_max',max_law_deviation,...
 'active_penalty_derivative_relative_error',d.audit.active_state,...
 'GNN_jacobian_relative_error',d.audit.GNN_jacobian,'date',char(datetime('now')));
f=fopen(fullfile(root,'results','verification.json'),'w');fprintf(f,'%s',jsonencode(report,'PrettyPrint',true));fclose(f);
disp(report);
end
