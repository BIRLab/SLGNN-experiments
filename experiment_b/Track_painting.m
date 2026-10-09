clear all;
format long;

 l1=90; l2=10; l3=10; l4=370; l5=310; l6=50;
 q_0 = 0;
 qr = [ 0;  0;  0;  0;  0;];    %1; 2; 3; 4; 5（关节号）; 
 ql = [ 0;  0;  0;  0;  0;];    %6; 7; 8; 9; 10;
 q=[q_0;qr;ql];
 
[xr, yr, zr, xl, yl, zl, T] = ztmkinematics(l1, l2, l3, l4 ,l5, l6, q);

t=1;

ch(1:length(t),1)=0;
ch(1:length(t),2)=0;
ch(1:length(t),3)=0.282;


figure;
 t = 1;

ch = [0, 0, 0]; % 身体中心点坐标

% 绘制身体中心点
scatter3(ch(1), ch(2), ch(3), 'filled', 'MarkerFaceColor', 'b');
hold on;

% 绘制右腿
plot3(xr, yr, zr, 'r', 'LineWidth', 2);

% 绘制左腿
plot3(xl, yl, zl, 'g', 'LineWidth', 2);

% 躯干
% 假设躯干长度为 trunk_length，位于身体中心点的上方
trunk_length = 450; % 躯干长度，根据实际情况修改
neck_length = 0; % 脖子长度，根据实际情况修改



% 绘制躯干
trunk_start = ch + [0, 0, 0]; % 躯干起点为身体中心的位置
trunk_end = trunk_start + [0, 0, trunk_length]; % 躯干终点在垂直方向上
plot3([trunk_start(1), trunk_end(1)], [trunk_start(2), trunk_end(2)], [trunk_start(3), trunk_end(3)], 'k', 'LineWidth', 3);

% 绘制头部
head_radius = 60; % 头部半径，根据实际情况修改
head_center = trunk_end + [0, 0, neck_length + head_radius]; % 头部中心点在躯干终点的上方
[x, y, z] = sphere;
x_head = head_radius * x + head_center(1);
y_head = head_radius * y + head_center(2);
z_head = head_radius * z + head_center(3);
surf(x_head, y_head, z_head, 'FaceColor', 'b', 'EdgeColor', 'none'); % 绘制头部

% 双臂
arm_length = 500; % 双臂长度，根据实际情况修改
shoulder_offset = 400; % 肩膀位置与身体中心点的垂直偏移量，根据实际情况修改

% 右臂
right_arm_start = ch + [0, 0, shoulder_offset]; % 右臂起点为身体中心点上方的位置
right_arm_end = right_arm_start + [0, arm_length, 0]; % 右臂终点在水平方向上
plot3([right_arm_start(1), right_arm_end(1)], [right_arm_start(2), right_arm_end(2)], [right_arm_start(3), right_arm_end(3)], 'm', 'LineWidth', 2);

% 左臂
left_arm_start = ch + [0, 0, shoulder_offset]; % 左臂起点为身体中心点上方的位置
left_arm_end = left_arm_start - [0, arm_length, 0]; % 左臂终点在水平方向上
plot3([left_arm_start(1), left_arm_end(1)], [left_arm_start(2), left_arm_end(2)], [left_arm_start(3), left_arm_end(3)], 'c', 'LineWidth', 2);

xlabel('X');
ylabel('Y');
zlabel('Z');
title('10自由度双足机器人');

axis equal;
grid on;
legend('身体中心点','右腿', '左腿', '躯干', '头部', '右臂', '左臂');