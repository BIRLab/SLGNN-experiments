function replot_source_data
% Inspect archived numeric source data. Does not rerun experiment_a--d.
root=fileparts(fileparts(mfilename('fullpath')));
figure('Name','Fig.4: archived numerical panels','Color','w');tiledlayout(2,2);
T=readtable(fullfile(root,'fig4','fig4c_feasible_region.csv'));nexttile;hold on
for m=["MVN","SLGNN"],idx=string(T.method)==m;plot(T.speed_over_MVN_boundary(idx),T.max_normalized_joint_velocity(idx),'-o','DisplayName',m);end
yline(1,'--');xlabel('Speed / MVN boundary');ylabel('Max normalized joint velocity');legend
T=readtable(fullfile(root,'fig4','fig4d.csv'));nexttile;plot(T{:,1},T{:,2:3});xlabel('Time (s)');ylabel('Slack rate (mm/s)');legend('Right','Left')
for name={'fig4e_MVN_joint_rates.csv','fig4f_SLGNN_joint_rates.csv'}
 T=readtable(fullfile(root,'fig4',name{1}));nexttile;plot(T{:,1},T{:,2:end});xlabel('Time (s)');ylabel('Joint rate (rad/s)');title(strrep(name{1},'_',' '));
end
figure('Name','Fig.5: five paired archived trials','Color','w');tiledlayout(1,2);
T=readtable(fullfile(root,'fig5','fig5a_median_residual.csv'));nexttile;semilogy(T{:,1},T{:,2:4});hold on;yline(1e-3,':');xlim([0 .15]);xlabel('Time (s)');ylabel('Scaled KKT residual');legend('GNN','ZNN','SLGNN')
T=readtable(fullfile(root,'fig5','fig5b_trial_metrics.csv'));nexttile;hold on;names=["GNN","ZNN","SLGNN"];
for m=1:3
 vals=T.late_residual_RMS(string(T.method)==names(m));scatter(m+linspace(-.08,.08,numel(vals)),vals);plot([m-.18 m+.18],[1 1]*median(vals),'k-');
end
set(gca,'YScale','log','XTick',1:3,'XTickLabel',names);ylabel('Residual RMS (1--10 s)')
figure('Name','Fig.6: archived numerical panels','Color','w');tiledlayout(3,2);
files={'fig6b_joint4.csv','fig6b_joint9.csv','fig6c.csv','fig6d.csv','fig6e_right_velocity_error.csv','fig6e_left_velocity_error.csv'};
labels={'Right knee (rad/s)','Left knee (rad/s)','Max normalized joint velocity','Slack rate (mm/s)','Right error (m/s)','Left error (m/s)'};
for i=1:6,T=readtable(fullfile(root,'fig6',files{i}));nexttile;plot(T{:,1},T{:,2:end});xlabel('Time (s)');ylabel(labels{i});end
for name={'fig4a','fig6a'}
 T=readtable(fullfile(root,name{1}(1:4),[name{1} '_saved_geometry.csv']));
 meta=jsondecode(fileread(fullfile(root,name{1}(1:4),[name{1} '_objects.json'])));
 figure('Name',[name{1} ': saved geometry'],'Color','w');hold on
 for i=1:numel(meta)
  if strcmp(meta(i).type,'line'),idx=T.object_id==i;plot3(T.x_mm(idx),T.y_mm(idx),T.z_mm(idx));end
 end
 xlabel('X (mm)');ylabel('Y (mm)');zlabel('Z (mm)');axis equal;grid on;view(3)
end
disp('Fig.4b/g/h: original numerical arrays unavailable; see native EPS/PDF and README.');
end
