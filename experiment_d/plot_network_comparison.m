function figures = plot_network_comparison(root, showFigures, includeGain)
% Arial, white background, paper palette, fine grids, a-f panel labels.
if nargin==0,root=fileparts(mfilename('fullpath'));end
d=load(fullfile(root,'results','comparison.mat'));
if nargin<2,showFigures=true;end
if nargin<3
 includeGain=~isfield(d,'mode') || strcmp(d.mode,'full');
end
figures=gobjects(0);
visibility='off';if showFigures,visibility='on';end
colors=[.18 .44 .62;.25 .57 .40;.74 .30 .18];styles={'--','-.','-'};
names={'GNN','ZNN','smg-RNN'};
if isfield(d.cfg,'methods'),names=d.cfg.methods;end
displayNames=names;displayNames(strcmp(displayNames,'smg-RNN'))={'SLGNN'};
fig=figure('Visible',visibility,'Color','w','Units','centimeters','Position',[2 2 18.3 18.5]);
tl=tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
for panel=1:6
 ax=nexttile(tl);hold(ax,'on');
 for m=1:3
  r=d.runs{1,m};
  switch panel
   case 1,xx=r.t;yy=r.residual;
   case 2,xx=r.t;yy=r.residual;
   case 3,xx=r.t;yy=r.qerror;
   case 4,xx=r.t;yy=r.task;
   case {5,6},continue;
  end
  plot(ax,xx,max(yy,1e-14),'Color',colors(m,:),'LineStyle',styles{m},'LineWidth',1.25,'DisplayName',displayNames{m});
 end
 switch panel
  case 1,xlim(ax,[0 .15]);ylim(ax,[1e-10 2]);ylabel(ax,'Scaled KKT residual');xlabel(ax,'Time (s)');set(ax,'YScale','log');
   yline(ax,1e-3,':','10^{-3}','Color',[.4 .4 .4],'LabelHorizontalAlignment','right','HandleVisibility','off');
   legend(ax,'Location','southwest','FontSize',7,'Box','off');
  case 2,xlim(ax,[0 10]);ylabel(ax,'Scaled KKT residual');xlabel(ax,'Time (s)');set(ax,'YScale','log');
  case 3,xlim(ax,[0 10]);ylabel(ax,'Joint-rate RMSE (rad s^{-1})');xlabel(ax,'Time (s)');set(ax,'YScale','log');
  case 4,xlim(ax,[0 10]);ylabel(ax,'Task residual (mm s^{-1})');xlabel(ax,'Time (s)');set(ax,'YScale','log');
  case 5
   b=bar(ax,1:3,d.summary.Wall_median_s,'FaceColor','flat','BarWidth',.58);b.CData=colors;
   for m=1:3
    u=d.trialTable.Wall_s(d.trialTable.Method==names{m});
    scatter(ax,m+linspace(-.10,.10,numel(u))',u,12,[.15 .15 .15],'filled');
   end
   set(ax,'XTick',1:3,'XTickLabel',displayNames);ylabel(ax,'Wall time for 10 s replay (s)');
  case 6
   vals=d.summary.Late_qdot_RMSE_median_rad_s;
   for m=1:3,plot(ax,m,max(vals(m),1e-14),'o','Color',colors(m,:),'MarkerFaceColor',colors(m,:),'MarkerSize',6);end
   set(ax,'YScale','log','XTick',1:3,'XTickLabel',displayNames);xlim(ax,[.5 3.5]);ylabel(ax,'Late joint-rate RMSE (rad s^{-1})');
 end
 style(ax);text(ax,-.16,1.08,char('a'+panel-1),'Units','normalized','FontName','Arial','FontSize',12,'FontWeight','bold');
end
exportall(fig,fullfile(root,'figures','network_comparison'));figures(end+1)=fig;
% Standalone figures for flexible LaTeX composition.
for panel=1:4
 fig=figure('Visible',visibility,'Color','w','Units','centimeters','Position',[2 2 9 5.7]);ax=axes(fig);hold(ax,'on');
 for m=1:3
  r=d.runs{1,m};
  switch panel,case {1,2},yy=r.residual;case 3,yy=r.qerror;case 4,yy=r.task;end
  plot(ax,r.t,max(yy,1e-14),'Color',colors(m,:),'LineStyle',styles{m},'LineWidth',1.4,'DisplayName',displayNames{m});
 end
 set(ax,'YScale','log');xlabel(ax,'Time (s)');
 switch panel
  case 1,xlim(ax,[0 .15]);ylabel(ax,'Scaled KKT residual');name='a_initial_convergence';
  case 2,xlim(ax,[0 10]);ylabel(ax,'Scaled KKT residual');name='b_tracking_residual';
  case 3,xlim(ax,[0 10]);ylabel(ax,'Joint-rate RMSE (rad s^{-1})');name='c_solution_accuracy';
  case 4,xlim(ax,[0 10]);ylabel(ax,'Task residual (mm s^{-1})');name='d_task_accuracy';
 end
 style(ax);legend(ax,'Location','best','Box','off','FontSize',8);exportall(fig,fullfile(root,'figures',name));figures(end+1)=fig;
end
gainfile=fullfile(root,'results','gain_sensitivity.csv');
if includeGain && isfile(gainfile)
 T=readtable(gainfile,'TextType','string');
 T.Method(T.Method=="smg-RNN")="SLGNN";
 fig=figure('Visible',visibility,'Color','w','Units','centimeters','Position',[2 2 18.3 6.3]);tl=tiledlayout(fig,1,2,'Padding','compact');
 for panel=1:2
  ax=nexttile(tl);hold(ax,'on');
  for m=1:3
   u=T(T.Method==displayNames{m},:);if panel==1,y=u.Late_qdot_RMSE_rad_s;else,y=u.Wall_s;end
   plot(ax,u.Gain,y,'-o','Color',colors(m,:),'LineStyle',styles{m},'MarkerSize',4,'LineWidth',1.3,'DisplayName',displayNames{m});
  end
  set(ax,'XScale','log','YScale','log','XTick',[10 30 100 300]);xlabel(ax,'Common gain');
  if panel==1,ylabel(ax,'Late joint-rate RMSE (rad s^{-1})');legend(ax,'Box','off','Location','best');else,ylabel(ax,'Wall time (s)');end
  style(ax);text(ax,-.13,1.08,char('a'+panel-1),'Units','normalized','FontWeight','bold','FontSize',12);
 end
 exportall(fig,fullfile(root,'figures','gain_sensitivity'));figures(end+1)=fig;
end
if showFigures,drawnow;end
end

function style(ax)
set(ax,'FontName','Arial','FontSize',8,'LineWidth',.7,'TickDir','in','Box','on',...
 'XGrid','on','YGrid','on','XMinorGrid','off','YMinorGrid','off',...
 'GridColor',[.80 .80 .80],'GridAlpha',.4,'MinorGridAlpha',.12);
end
function exportall(fig,path)
exportgraphics(fig,[path '.pdf'],'ContentType','vector','BackgroundColor','white');
exportgraphics(fig,[path '.png'],'Resolution',450,'BackgroundColor','white');
savefig(fig,[path '.fig']);
end
