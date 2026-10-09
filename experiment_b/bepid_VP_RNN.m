function doty = bepid_VP_RNN(t,y)
%   BEPID_VP_RNN 
%   神经动力学微分方程构建
%   2024.4.24

global  n m k p sigma gama dotX_max dotX_min;  


%% 系统当前值
q = y([1:4, 6:9]);%获取角度 y=[q_r(4项), delta_r(1), q_l(4项), delta_l(1), dotq_r(4项), dotdelta_r(1), dotq_l(4项), dotdelta_l(1),lamuda(6)]
dotq = y([n+1:n+4, n+6:n+9]);   %获取角速度
PS_lambda = y(2*n + 1:end);  %拉格朗日乘子λ
%松弛变量
delta_r = y(5);
delta_l = y(10);
dot_delta_r = y(15);
dot_delta_l = y(20);

X = [y(1:4); 0; y(6:9); 0]; %角度层
dotX = [y(n+1:n+4); dot_delta_r; y(n+6:n+9); dot_delta_l];  %速度层

%% 计算雅可比矩阵和其导数
[Jw,dot_Jw,Jw_aug,dot_Jw_aug] = ztm_jacobian(q,dotq);
%dot_Jw = zeros(6,8);

%% 轨迹设计

%圆形轨迹
% r = 90;
% Rw = [ r * sin(t + 3*pi/2) + 10; 0; r * cos(t + 3*pi/2) - 10; ...
%       -r * cos(t + 3*pi/2) + 10; 0; r * sin(t + 3*pi/2) - 10];
% 
% dotRw = [ r * cos(t + 3*pi/2); 0; -r * sin(t + 3*pi/2); ...
%           r * sin(t + 3*pi/2); 0;  r * cos(t + 3*pi/2)];
% 
% ddotRw = [ -r * sin(t + 3*pi/2); 0; -r * cos(t + 3*pi/2); ...
%             r * cos(t + 3*pi/2); 0;  r * sin(t + 3*pi/2)];

%原地不动
% dotRw = zeros(6,1);
% ddotRw = zeros(6,1);

%原地抬腿
 % dotRw = [ 0; 0; 40; ...
 %           0; 0; 40];
 % ddotRw = zeros(6,1);

% 世界坐标系下摆线轨迹
T_trajectory = 2; %完成一个步态周期时间
L_traj = 800; %步x轴长度

x0_r = 0; x1_r = x0_r + L_traj;  x0_l = 0; x1_l = x0_l + L_traj;% x 范围
y0_r = 0; y1_r = 0; y0_l = 0; y1_l = 0;% y 范围
z0_r = 0; z0_l = 0; h_traj = 240;   % z 范围,步态高度

%身体中心轨迹
xpos_body = L_traj/T_trajectory;
ypos_body = zeros(1);
zpos_body = zeros(1);


k_bodyspeed = 1;%身体速度调节系数
dotxpos_body = L_traj/(T_trajectory * 2);
ddotxpos_body = 0;

%计算相位
time_flag = t;%记录该时刻的时间点
while(time_flag > T_trajectory)
    time_flag = time_flag - T_trajectory;
end
p_traj = time_flag/T_trajectory; 

%足端轨迹
 if p_traj <= 0.5 %步态周期前半段
     p_traj = p_traj * 2;
    %摆线轨迹位置方程
            x_r_curve = x0_r + (x1_r - x0_r) / (2 * pi) * (2 * pi * p_traj - sin(2 * pi * p_traj));
            y_r_curve = y0_r + (y1_r - y0_r) / (2 * pi) * (2 * pi * p_traj - sin(2 * pi * p_traj));
            z_r_curve = z0_r + h_traj /2 * (1 - cos(2 * pi * p_traj));

 Rw = [     x_r_curve - xpos_body;   y_r_curve;    z_r_curve;...
            0;   0;    0];

    %摆线轨迹速度方程
            dotx_r_curve = (x1_r - x0_r) / T_trajectory * (1 - cos(2 * pi * p_traj));
            doty_r_curve = (y1_r - y0_r) / T_trajectory * (1 - cos(2 * pi * p_traj));
            dotz_r_curve = pi*h_traj / T_trajectory * sin(2 * pi * p_traj);

 dotRw = [  dotx_r_curve-dotxpos_body;   doty_r_curve;    dotz_r_curve + 78;...
            -dotxpos_body;   0;    0 - 41];

    %摆线轨迹加速度方程
            ddotx_r_curve = 2*pi*(x1_r - x0_r) / T_trajectory^2 * (sin(2 * pi * p_traj));
            ddoty_r_curve = 2*pi*(y1_r - y0_r) / T_trajectory^2 * (sin(2 * pi * p_traj));
            ddotz_r_curve = 2*pi^2*h_traj / T_trajectory^2 * cos(2 * pi * p_traj);

ddotRw = [  ddotx_r_curve;   ddoty_r_curve;    ddotz_r_curve;...
            0;   0;    0];

else  %步态周期后半段脚触地不动
     p_traj = p_traj - 0.5;
     p_traj = p_traj * 2;
    %左腿摆线轨迹速度方程
            dotx_l_curve = (x1_l - x0_l) / T_trajectory * (1 - cos(2 * pi * p_traj));
            doty_l_curve = (y1_l - y0_l) / T_trajectory * (1 - cos(2 * pi * p_traj));
            dotz_l_curve = pi*h_traj / T_trajectory * sin(2 * pi * p_traj);

 dotRw = [  -dotxpos_body;   0;    0 - 40;...
            dotx_l_curve-dotxpos_body;   doty_l_curve;    dotz_l_curve + 40];

  %摆线轨迹加速度方程
            ddotx_l_curve = 2*pi*(x1_l - x0_l) / T_trajectory^2 * (sin(2 * pi * p_traj));
            ddoty_l_curve = 2*pi*(y1_l - y0_l) / T_trajectory^2 * (sin(2 * pi * p_traj));
            ddotz_l_curve = 2*pi^2*h_traj / T_trajectory^2 * cos(2 * pi * p_traj);

ddotRw = [  0;   0;    0 ;...
            ddotx_l_curve;   ddoty_l_curve;    ddotz_l_curve ];

 end

dot_Rw_slack = dotRw;
ddot_Rw_slack = ddotRw;

%% TVQP-原问题-参数矩阵

QP_H = eye(n,n); 
QP_H(5, 5) = 0.2;
QP_H(10, 10) = 0.2;

QP_K = Jw_aug;         %雅可比矩阵  注意！雅可比矩阵只求解到脚跟处，未加入5号关节角度，故单腿矩阵大小4*3+delta
QP_b = zeros(n,1);
QP_c = dot_Rw_slack;      %轨迹导数
QP_d = [dotX_max ; - dotX_min];
QP_J = [eye(n); -eye(n)];
QP_x = dotX;
QP_delta = [0; 0; dot_delta_r; 0; 0; dot_delta_l];

dot_QP_H = [zeros(n)];%由于两项皆为常数直接取零
dot_QP_b = zeros(n,1);
dot_QP_c = ddot_Rw_slack;
dot_QP_J = [zeros(n) ; zeros(n)]; 
dot_QP_d = [zeros(n,1) ; zeros(n,1)];
dot_QP_K = dot_Jw_aug;
% dot_QP_delta = [0; 0; dot_delta_r; 0; 0; dot_delta_l];
dot_QP_delta = [0; 0; 0; 0; 0; 0];
%% TVQP-引入惩罚函数-拉格朗日乘子-参数矩阵

%计算惩罚累加项
[PS_Pexp_g,PS_Pexp_Q,PS_Pexp_S,PS_Pexp_u] = PS_Pexp(QP_d,QP_J,QP_x,dot_QP_J,dot_QP_d);
%delta对X的偏导
J_delta = zeros(6, 10);
J_delta(3, 5) = 1;
J_delta(6, 10) = 1;

PS_A = [ QP_H,   QP_K' - J_delta'; ...      %对应A（t）
         QP_K, zeros(k,k)];

PS_y = [   QP_x;...             %对应y（t）
         PS_lambda];

PS_g = [ -QP_b - p*sigma*PS_Pexp_g;...   %对应g（t）
         QP_c + QP_delta];


%% PS_VP_RNN solver

RNN_Q = [ QP_H + p*sigma^2*PS_Pexp_Q,     QP_K' ;...
                QP_K           zeros(k,k)  ];

RNN_S = [ dot_QP_H + p*sigma^2*PS_Pexp_S,   dot_QP_K' ;...
               dot_QP_K           zeros(k,k)  ];

RNN_u = [ -dot_QP_b - p*sigma*PS_Pexp_u;...
                  dot_QP_c + dot_QP_delta     ];

% flag_123 = (RNN_Q' * RNN_Q + ones(size(RNN_Q)));
%rank_Q = rank(RNN_Q);              % 计算矩阵的秩
dotyy = RNN_Q\ (-RNN_S*PS_y - gama*exp(t)*(PS_A*PS_y - PS_g) + RNN_u);          %PS_VP_RNN
flag_2 = PS_A*PS_y - PS_g;
flag_3 = exp(t)*(PS_A*PS_y - PS_g);
doty = [dotX; dotyy];

% Keep the ODE call quiet; the old bare "t" printed every solver step.
t;
end

