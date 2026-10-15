function export_source_data
% Export saved figure arrays and archived solver runs; no simulation is rerun.
root=fileparts(fileparts(mfilename('fullpath')));
native=fullfile(root,'native');
for name={'fig4','fig5','fig6','tables'},if ~isfolder(fullfile(root,name{1})),mkdir(fullfile(root,name{1}));end,end
geometry(native,root,'fig4a','fig4');
geometry(native,root,'fig6a','fig6');
joint_rates(native,root,'fig4e','MVN');
joint_rates(native,root,'fig4f','SLGNN');
paired(native,root,'fig4d','fig4',{'Right','Left'},'slack_rate_mm_s');
paired(native,root,'fig6d','fig6',{'Right','Left'},'slack_rate_mm_s');
paired(native,root,'fig6b_joint4','fig6',{'MVN','SLGNN'},'q4_rate_rad_s');
paired(native,root,'fig6b_joint9','fig6',{'MVN','SLGNN'},'q9_rate_rad_s');
paired(native,root,'fig6c','fig6',{'MVN','SLGNN'},'max_normalized_joint_velocity');
f=openfig(fullfile(native,'fig4c.fig'),'invisible');cleanup=onCleanup(@()close(f));
a=findall(f,'Type','axes');lines=findall(a,'Type','line');rows=cell(0,4);
for i=1:numel(lines)
 h=lines(i);method=string(h.DisplayName);p=0;if method=="SLGNN",p=200;end
 for j=1:numel(h.XData),rows(end+1,:)={method,p,h.XData(j),h.YData(j)};end %#ok<AGROW>
end
T=cell2table(rows,'VariableNames',{'method','penalty_p','speed_over_MVN_boundary','max_normalized_joint_velocity'});
writetable(T,fullfile(root,'fig4','fig4c_feasible_region.csv'));
h=findall(a,'Type','patch');write_numeric(fullfile(root,'fig4','fig4c_shaded_region.csv'),{'vertex','speed_over_MVN_boundary','normalized_velocity'},[(1:numel(h.XData))',h.XData(:),h.YData(:)]);
clear cleanup
for side={'right','left'}
 name=['fig6e_' side{1}];f=openfig(fullfile(native,[name '.fig']),'invisible');cleanup=onCleanup(@()close(f));
 ax=findall(f,'Type','axes');[~,idx]=max(arrayfun(@(a)diff(a.XLim),ax));a=ax(idx);
 h=findall(a,'Type','line');x=find(strcmp({h.DisplayName},'$e_x$'));y=find(strcmp({h.DisplayName},'$e_y$'));v=find(strcmp({h.DisplayName},'$\|e\|$'));
 assert(numel(x)==1 && numel(y)==1 && numel(v)==1);
 t=h(x).XData(:);data=[t,h(x).YData(:),h(y).YData(:),h(v).YData(:)];
 assert(isequal(t,h(y).XData(:)) && isequal(t,h(v).XData(:)));
 assert(max(abs(hypot(data(:,2),data(:,3))-data(:,4)))<1e-12);
 write_numeric(fullfile(root,'fig6',[name '_velocity_error.csv']),{'time_s','error_x_m_s','error_y_m_s','error_norm_m_s'},data);
 clear cleanup
end
% Original five paired trials used for manuscript Fig.5 and Table3.
d=load(fullfile(native,'fig5_comparison.mat'));
names={'GNN','ZNN','SLGNN'};t=d.runs{1,1}.t(:);all=[t];headers={'time_s'};med=t;medheaders={'time_s'};
rows=cell(0,6);summary=cell(0,6);
for m=1:3
 R=zeros(numel(t),5);rms=zeros(5,1);tc=nan(5,1);
 for j=1:5
  r=d.runs{j,m};assert(isequal(r.t(:),t));R(:,j)=r.residual(:);
  late=t>=1 & t<=10;rms(j)=sqrt(trapz(t(late),R(late,j).^2)/9);
  ii=find(R(:,j)<=1e-3,1);if ~isempty(ii),tc(j)=t(ii);end
  assert(abs(rms(j)-r.metrics.Late_residual_RMS)<1e-12);
  assert(isequaln(tc(j),r.metrics.First_1e3_s));
  assert(norm(r.y(1,:)'-d.initials(:,j))<1e-12);
  rows(end+1,:)={names{m},j,tc(j),rms(j),1,10}; %#ok<AGROW>
  headers{end+1}=sprintf('%s_trial%d',names{m},j); %#ok<AGROW>
 end
 all=[all,R];med=[med,median(R,2)];medheaders{end+1}=names{m}; %#ok<AGROW>
 summary(end+1,:)={names{m},5,median(tc),median(rms),1,10}; %#ok<AGROW>
end
write_numeric(fullfile(root,'fig5','fig5a_all_residuals.csv'),headers,all);
write_numeric(fullfile(root,'fig5','fig5a_median_residual.csv'),medheaders,med);
T=cell2table(rows,'VariableNames',{'method','trial','first_threshold_time_s','late_residual_RMS','window_start_s','window_end_s'});writetable(T,fullfile(root,'fig5','fig5b_trial_metrics.csv'));
T=cell2table(summary,'VariableNames',{'method','paired_trials','median_threshold_time_s','median_late_residual_RMS','window_start_s','window_end_s'});writetable(T,fullfile(root,'tables','table3_solver_summary.csv'));
assert(abs(T.median_late_residual_RMS(1)-.13133472317768138)<1e-14);
assert(abs(T.median_late_residual_RMS(2)-4.877559193598539e-6)<1e-16);
assert(abs(T.median_late_residual_RMS(3)-6.159872184545438e-7)<1e-17);
write_numeric(fullfile(root,'fig5','paired_initial_states.csv'),{'state_index','trial1','trial2','trial3','trial4','trial5'},[(1:16)',d.initials]);
cfg=d.cfg;cfg=rmfield(cfg,'t');cfg.methods=names;cfg.integrator='ode15s';cfg.source='Archived paper results; no rerun';
fid=fopen(fullfile(root,'fig5','solver_configuration.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(cfg,'PrettyPrint',true));fclose(fid);
disp('Archived source arrays exported and Fig.5 statistics verified.');
end

function joint_rates(native,root,name,method)
f=openfig(fullfile(native,[name '.fig']),'invisible');cleanup=onCleanup(@()close(f));
a=findall(f,'Type','axes');h=findall(a,'Type','line');ids=[1:4,6:9];data=[];headers={'time_s'};
for j=1:8
 idx=find(strcmp({h.DisplayName},sprintf('Joint %d',ids(j))));assert(numel(idx)==1);
 if j==1,t=h(idx).XData(:);data=t;else,assert(isequal(t,h(idx).XData(:)));end
 data(:,j+1)=h(idx).YData(:);headers{end+1}=sprintf('q%d_rate_rad_s',ids(j)); %#ok<AGROW>
end
assert(size(data,1)==1001);write_numeric(fullfile(root,'fig4',[name '_' method '_joint_rates.csv']),headers,data);
end

function paired(native,root,name,folder,keys,quantity)
f=openfig(fullfile(native,[name '.fig']),'invisible');cleanup=onCleanup(@()close(f));
a=findall(f,'Type','axes');h=findall(a,'Type','line');data=[];headers={'time_s'};
for j=1:2
 labels={h.DisplayName};idx=find(contains(labels,keys{j}));
 if isempty(idx) && startsWith(quantity,'slack')
  if j==1,idx=find(contains(labels,'delta}_R'));else,idx=find(contains(labels,'delta}_L'));end
 end
 assert(numel(idx)==1,[name ': ambiguous curve']);
 if j==1,t=h(idx).XData(:);data=t;else,assert(isequal(t,h(idx).XData(:)));end
 data(:,j+1)=h(idx).YData(:);headers{end+1}=[keys{j} '_' quantity]; %#ok<AGROW>
end
assert(size(data,1)==1001);write_numeric(fullfile(root,folder,[name '.csv']),headers,data);
end

function geometry(native,root,name,folder)
f=openfig(fullfile(native,[name '.fig']),'invisible');cleanup=onCleanup(@()close(f));
ax=findall(f,'Type','axes');h=findall(ax,'-property','XData');data=[];meta=struct([]);
for i=1:numel(h)
 x=h(i).XData;y=h(i).YData;z=h(i).ZData;
 assert(isequal(size(x),size(y)) && isequal(size(x),size(z)));
 [rr,cc]=ndgrid(1:size(x,1),1:size(x,2));
 data=[data;repmat(i,numel(x),1),(1:numel(x))',rr(:),cc(:),x(:),y(:),z(:)]; %#ok<AGROW>
 meta(i).object_id=i;meta(i).type=h(i).Type;meta(i).display_name=h(i).DisplayName;meta(i).array_size=size(x);
end
write_numeric(fullfile(root,folder,[name '_saved_geometry.csv']),{'object_id','vertex_index','matrix_row','matrix_column','x_mm','y_mm','z_mm'},data);
fid=fopen(fullfile(root,folder,[name '_objects.json']),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(meta,'PrettyPrint',true));fclose(fid);
end

function write_numeric(path,headers,data)
% 17 significant decimal digits preserve IEEE-754 doubles on round trip.
fid=fopen(path,'w');cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',strjoin(headers,','));fmt=[repmat('%.17g,',1,size(data,2)-1) '%.17g\n'];fprintf(fid,fmt,data');clear cleanup
check=readmatrix(path);assert(isequaln(check,data),['CSV roundtrip failed: ' path]);
end
