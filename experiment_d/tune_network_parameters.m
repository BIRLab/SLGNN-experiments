function tune_network_parameters(mode)
% Parameter-only comparisons: unchanged QP, tolerances, integrator and scaling.
if nargin==0,mode='scan';end
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'src'));
out=fullfile(root,'tuning');if ~isfolder(out),mkdir(out);end
diary(fullfile(out,['log_' mode '.txt']));clean=onCleanup(@()diary('off'));
a=run_network_comparison('api');d=load(fullfile(root,'results','comparison.mat'));
load(fullfile(root,'inputs','common_robot_replay.mat'),'input');
P.q=spline(input.t,input.q');P.c=spline(input.t,input.c');P.dq=a.ppderivative(P.q);P.dc=a.ppderivative(P.c);
cfg=d.cfg;cfg.beta=1;ref=d.ref;y0=zeros(16,1);
global l1 l2 l3 l4 l5 l6;
l1=90;l2=0;l3=10;l4=370;l5=310;l6=100;
cc=cfg;cc.T=.02;
for name={'GNN','ZNN','smg-RNN'},a.solve(name{1},P,cc,y0);end
if strcmp(mode,'scan')
 settings=struct([]);
 for gain=[30 100 300 1000]
  for name={'GNN','ZNN'}
   st=struct('Method',string(name{1}),'Gamma',gain,'Beta',0);
   if isempty(settings),settings=st;else,settings(end+1)=st;end %#ok<AGROW>
  end
  for beta=[.05 .1 .2 .4 1]
   settings(end+1)=struct('Method',"smg-RNN",'Gamma',gain,'Beta',beta); %#ok<AGROW>
  end
 end
 rng(20260910);order=randperm(numel(settings));rows=struct([]);
 for j=order
  st=settings(j);cc=cfg;cc.gamma=st.Gamma;cc.beta=st.Beta;
  r=a.measure(a.solve(char(st.Method),P,cc,y0),P,cc,ref);
  row=r.metrics;row.Gamma=st.Gamma;row.Beta=st.Beta;
  if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
  fprintf('%s gamma=%g beta=%g wall=%.3f residual=%.3g qerr=%.3g tc=%.3f\n',...
   st.Method,st.Gamma,st.Beta,row.Wall_s,row.Late_residual_RMS,row.Late_qdot_RMSE_rad_s,row.First_1e3_s);
  writetable(struct2table(rows),fullfile(out,'scan.csv'));
 end
 save(fullfile(out,'scan.mat'),'rows','settings','cfg','order');
elseif strcmp(mode,'confirm')
 % The candidate list is explicit and selected after examining scan.csv.
 settings=readtable(fullfile(out,'candidates.csv'),'TextType','string');
 rng(20260911);ys=[zeros(16,1),.02*randn(16,4)];rows=struct([]);runs=cell(5,height(settings));
 for trial=1:5
  for j=circshift(1:height(settings),[0 mod(trial-1,height(settings))])
   st=settings(j,:);cc=cfg;cc.gamma=st.Gamma;cc.beta=st.Beta;
   r=a.measure(a.solve(char(st.Method),P,cc,ys(:,trial)),P,cc,ref);
   row=r.metrics;row.Gamma=st.Gamma;row.Beta=st.Beta;row.Trial=trial;row.Label=st.Label;
   if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
   runs{trial,j}=r;
   fprintf('Confirm %d %s: %.3fs residual %.3g qerr %.3g\n',trial,st.Label,row.Wall_s,row.Late_residual_RMS,row.Late_qdot_RMSE_rad_s);
   writetable(struct2table(rows),fullfile(out,'confirmation.csv'));
  end
 end
 save(fullfile(out,'confirmation.mat'),'rows','runs','ys','settings','cfg','-v7.3');
else,error('Unknown mode');end
fprintf('DONE %s\n',char(datetime('now')));
end
