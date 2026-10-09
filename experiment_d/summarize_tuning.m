function summarize_tuning
root=fileparts(mfilename('fullpath'));out=fullfile(root,'tuning');
T=readtable(fullfile(out,'confirmation.csv'),'TextType','string');
d=load(fullfile(out,'confirmation.mat'));rows=struct([]);
for j=1:height(d.settings)
 st=d.settings(j,:);U=sortrows(T(T.Label==st.Label,:),'Trial');
 s.Label=st.Label;s.Method=st.Method;s.Gamma=st.Gamma;s.Beta=st.Beta;s.N=height(U);
 s.Wall_median_s=median(U.Wall_s);s.Wall_min_s=min(U.Wall_s);s.Wall_max_s=max(U.Wall_s);
 s.Residual_median=median(U.Late_residual_RMS);s.Qdot_RMSE_median=median(U.Late_qdot_RMSE_rad_s);
 s.Threshold_median_s=median(U.Sustained_1e3_s);s.RHS_median=median(U.RHS_calls);
 if isempty(rows),rows=s;else,rows(end+1)=s;end %#ok<AGROW>
end
S=struct2table(rows);writetable(S,fullfile(out,'summary.csv'));disp(S);
% Validate threshold dynamics and metrics for all paired runs.
base=load(fullfile(root,'results','comparison.mat'),'ref');lawError=0;metricError=0;
for trial=1:5
 for j=1:height(d.settings)
  r=d.runs{trial,j};st=d.settings(j,:);assert(r.t(end)==10 && all(isfinite(r.y),'all'));
  assert(norm(r.y(1,:)'-d.ys(:,trial))<1e-12);
  if st.Method~="GNN"
   mask=r.t<=.05;
   if st.Beta==0,integral=st.Gamma*r.t(mask);
   else,integral=st.Gamma/st.Beta*expm1(st.Beta*r.t(mask));end
   lawError=max(lawError,max(abs(r.residual(mask)-r.residual(1)*exp(-integral)))/max(1,r.residual(1)));
  end
  mask=r.t>=1;qi=[1:4 6:9];e2=mean((r.x(:,qi)-base.ref.x(:,qi)).^2,2);
  re=sqrt(trapz(r.t(mask),e2(mask))/9);metricError=max(metricError,abs(re-r.metrics.Late_qdot_RMSE_rad_s));
 end
end
assert(lawError<1e-4 && metricError<1e-12);
V=struct('passed',true,'paired_runs',35,'metric_error',metricError,'decay_law_error',lawError);
fid=fopen(fullfile(out,'verification.json'),'w');fprintf(fid,'%s',jsonencode(V,'PrettyPrint',true));fclose(fid);
% Pairwise outcomes: no pooling single-shot scan times with confirmation times.
A=sortrows(T(T.Label=="smg_g1000_b01",:),'Trial');pairs=struct([]);
for label=["smg_original","ZNN_original","ZNN_gain1000","GNN_gain300"]
 B=sortrows(T(T.Label==label,:),'Trial');p.Comparator=label;
 p.Faster_runs=sum(A.Wall_s<B.Wall_s);p.Lower_residual_runs=sum(A.Late_residual_RMS<B.Late_residual_RMS);
 p.Lower_qdot_error_runs=sum(A.Late_qdot_RMSE_rad_s<B.Late_qdot_RMSE_rad_s);
 p.Wall_ratio=median(A.Wall_s)/median(B.Wall_s);p.Residual_ratio=median(A.Late_residual_RMS)/median(B.Late_residual_RMS);
 p.Qdot_error_ratio=median(A.Late_qdot_RMSE_rad_s)/median(B.Late_qdot_RMSE_rad_s);
 if isempty(pairs),pairs=p;else,pairs(end+1)=p;end %#ok<AGROW>
end
writetable(struct2table(pairs),fullfile(out,'paired_comparisons.csv'));disp(struct2table(pairs));
% Paper-style focused before/after figure.
sel=[1 3 4 5];colors=[.18 .44 .62;.25 .57 .40;.65 .65 .65;.74 .30 .18];
names={'ZNN, g=100','ZNN, g=1000','smg-RNN, original','smg-RNN, tuned'};
ticks={'ZNN 100','ZNN 1000','smg original','smg tuned'};lines={'--','-.',':','-'};
fig=figure('Visible','off','Color','w','Units','centimeters','Position',[2 2 18.3 12.8]);
tl=tiledlayout(fig,2,2,'Padding','compact','TileSpacing','compact');
for panel=1:4
 ax=nexttile(tl);hold(ax,'on');
 if panel==1
  for m=1:4,r=d.runs{1,sel(m)};plot(ax,r.t,max(r.residual,1e-14),'LineWidth',1.25,'Color',colors(m,:),'LineStyle',lines{m},'DisplayName',names{m});end
  set(ax,'YScale','log');xlim(ax,[0 .1]);ylim(ax,[1e-11 2]);xlabel(ax,'Time (s)');ylabel(ax,'Scaled KKT residual');
  yline(ax,1e-3,':','HandleVisibility','off','Color',[.4 .4 .4]);
  lg=legend(ax,'Box','off','FontSize',8,'Orientation','horizontal');lg.Layout.Tile='north';
 else
  for m=1:4
   U=T(T.Label==d.settings.Label(sel(m)),:);
   if panel==2,v=U.Wall_s;elseif panel==3,v=U.Late_residual_RMS;else,v=U.Late_qdot_RMSE_rad_s;end
   plot(ax,[m-.17 m+.17],median(v)*[1 1],'-','Color',colors(m,:),'LineWidth',2);
   scatter(ax,m+linspace(-.10,.10,numel(v))',v,15,colors(m,:),'filled');
  end
  set(ax,'XTick',1:4,'XTickLabel',ticks,'XTickLabelRotation',15);xlim(ax,[.5 4.5]);
  if panel==2,ylabel(ax,'Wall time for 10 s replay (s)');
  elseif panel==3,ylabel(ax,'Late scaled residual RMS');set(ax,'YScale','log','YTick',[5e-7 1e-6 5e-6]);ylim(ax,[3e-7 7e-6]);
  else,ylabel(ax,'Late joint-rate RMSE (rad s^{-1})');end
 end
 set(ax,'FontName','Arial','FontSize',8,'LineWidth',.7,'TickDir','in','Box','on',...
  'XGrid','on','YGrid','on','XMinorGrid','off','YMinorGrid','off','GridColor',[.8 .8 .8],'GridAlpha',.4);
 text(ax,-.14,1.08,char('a'+panel-1),'Units','normalized','FontName','Arial','FontSize',12,'FontWeight','bold');
end
exportgraphics(fig,fullfile(out,'tuning_comparison.pdf'),'ContentType','vector','BackgroundColor','white');
exportgraphics(fig,fullfile(out,'tuning_comparison.png'),'Resolution',450,'BackgroundColor','white');
savefig(fig,fullfile(out,'tuning_comparison.fig'));close(fig);
end
