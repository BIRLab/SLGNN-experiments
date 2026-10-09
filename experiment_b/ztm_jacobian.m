function [Jw,dot_Jw,Jw_aug,dot_Jw_aug] = ztm_jacobian(q,dotq)
%ZTM_JACOBIAN 此处显示有关此函数的摘要
%   此函数用于计算8自由度（不计算脚掌自由度）的雅可比矩阵及其导数

global l1 l2 l3 l4 l5 l6;

%%  调试用参数
% l1=90; l2=0; l3=100; l4=370; l5=310; l6=100;
% q_0 = pi/2;%0;
 % qr = [ 0;  0;  0;  0; ];%1; 2; 3; 4; 5（关节号）; 
 % ql = [ 0;  pi/8;  pi/4;  -pi/5;];%6; 7; 8; 9; 10;
 % q=[qr;ql];

 % dq1 = 1.2; dq2 = 0.8; dq3 = 0.4; dq4 = 1.8; dq5 = 1.2; dq6 = 0.6; dq7 = 2.2; dq8 = 2.1; dq9 = 0.1;
 % dotq = [1.2; 0.8; 0.4; 1.8; 1.2; 0.6; 1.2; 2.1];
%% 三角函数求解
  allsin=sin(q);
% 腰部
    %s0=allsin(1);
% 右腿
    s1= allsin(1);%1号关节
    s2= allsin(2);
    s3= allsin(3);
    s4= allsin(4); 
    %s5= allsin(6);
% 左腿
    s6= allsin(5);%6号关节
    s7= allsin(6);
    s8= allsin(7);
    s9= allsin(8);
    %s10=allsin(11);

 allcos=cos(q);
% 腰部
%    c0= allcos(1);
% 右腿
    c1= allcos(1);
    c2= allcos(2);
    c3= allcos(3);
    c4= allcos(4);
 %   c5= allcos(6);
 % 左腿
    c6= allcos(5);
    c7= allcos(6);   
    c8= allcos(7);
    c9= allcos(8);
 %   c10=allcos(11);

%% 取出角速度
dq1 = dotq(1); dq2 = dotq(2); dq3 = dotq(3); dq4 = dotq(4);
dq6 = dotq(5); dq7 = dotq(6); dq8 = dotq(7); dq9 = dotq(8); 

%%  雅可比矩阵计算

%右腿雅可比
J_r_11 = 0;
J_r_12 = l5*(c3*s2*s4 + c4*s2*s3) + l4*s2*s3;
J_r_13 = -l5*(c2*c3*c4 - c2*s3*s4) - l4*c2*c3;
J_r_14 = -l5*(c2*c3*c4 - c2*s3*s4);

J_r_21 = l4*(c1*c3 + s1*s2*s3) + l5*(c4*(c1*c3 + s1*s2*s3) - s4*(c1*s3 - c3*s1*s2)) + l3*c1;
J_r_22 = -l5*(c1*c2*c3*s4 + c1*c2*c4*s3) - l4*c1*c2*s3;
J_r_23 = -l4*(s1*s3 + c1*c3*s2) - l5*(c4*(s1*s3 + c1*c3*s2) + s4*(c3*s1 - c1*s2*s3));
J_r_24 = -l5*(c4*(s1*s3 + c1*c3*s2) + s4*(c3*s1 - c1*s2*s3));

J_r_31 = l4*(c3*s1 - c1*s2*s3) + l5*(c4*(c3*s1 - c1*s2*s3) - s4*(s1*s3 + c1*c3*s2)) + l3*s1;
J_r_32 = -l5*(c2*c3*s1*s4 + c2*c4*s1*s3) - l4*c2*s1*s3;
J_r_33 = l4*(c1*s3 - c3*s1*s2) + l5*(c4*(c1*s3 - c3*s1*s2) + s4*(c1*c3 + s1*s2*s3));
J_r_34 = l5*(c4*(c1*s3 - c3*s1*s2) + s4*(c1*c3 + s1*s2*s3));

J_r_value = [J_r_11, J_r_12, J_r_13, J_r_14;...
             J_r_21, J_r_22, J_r_23, J_r_24;...
             J_r_31, J_r_32, J_r_33, J_r_34];

%左腿雅可比

J_l_11 = 0;
J_l_12 = l5*(c8*s7*s9 + c9*s7*s8) + l4*s7*s8;
J_l_13 = -l5*(c7*c8*c9 - c7*s8*s9) - l4*c7*c8;
J_l_14 = -l5*(c7*c8*c9 - c7*s8*s9);

J_l_21 = l4*(c6*c8 + s6*s7*s8) + l5*(c9*(c6*c8 + s6*s7*s8) - s9*(c6*s8 - c8*s6*s7)) + l3*c6;
J_l_22 = -l5*(c6*c7*c8*s9 + c6*c7*c9*s8) - l4*c6*c7*s8;
J_l_23 = -l4*(s6*s8 + c6*c8*s7) - l5*(c9*(s6*s8 + c6*c8*s7) + s9*(c8*s6 - c6*s7*s8));
J_l_24 = -l5*(c9*(s6*s8 + c6*c8*s7) + s9*(c8*s6 - c6*s7*s8));

J_l_31 = l4*(c8*s6 - c6*s7*s8) + l5*(c9*(c8*s6 - c6*s7*s8) - s9*(s6*s8 + c6*c8*s7)) + l3*s6;
J_l_32 = -l5*(c7*c8*s6*s9 + c7*c9*s6*s8) - l4*c7*s6*s8;
J_l_33 = l4*(c6*s8 - c8*s6*s7) + l5*(c9*(c6*s8 - c8*s6*s7) + s9*(c6*c8 + s6*s7*s8));
J_l_34 = l5*(c9*(c6*s8 - c8*s6*s7) + s9*(c6*c8 + s6*s7*s8));

J_l_value = [J_l_11, J_l_12, J_l_13, J_l_14;...
            J_l_21, J_l_22, J_l_23, J_l_24;...
            J_l_31, J_l_32, J_l_33, J_l_34];

%% 计算雅可比矩阵导数

%右腿雅可比导数
dotJ_r_11 = 0;
dotJ_r_12 = dq2*(l5*(c2*c3*s4 + c2*c4*s3) + l4*c2*s3) - dq3*(l5*(s2*s3*s4 - c3*c4*s2) - l4*c3*s2) - dq4*l5*(s2*s3*s4 - c3*c4*s2);
dotJ_r_13 = dq3*(l5*(c2*c3*s4 + c2*c4*s3) + l4*c2*s3) - dq2*(l5*(s2*s3*s4 - c3*c4*s2) - l4*c3*s2) + dq4*l5*(c2*c3*s4 + c2*c4*s3);
dotJ_r_14 = dq3*l5*(c2*c3*s4 + c2*c4*s3) + dq4*l5*(c2*c3*s4 + c2*c4*s3) - dq2*l5*(s2*s3*s4 - c3*c4*s2);

dotJ_r_21 = dq2*(l5*(c2*c3*s1*s4 + c2*c4*s1*s3) + l4*c2*s1*s3) - dq1*(l4*(c3*s1 - c1*s2*s3) + l5*(c4*(c3*s1 - c1*s2*s3) - s4*(s1*s3 + c1*c3*s2)) + l3*s1) - dq3*(l4*(c1*s3 - c3*s1*s2) + l5*(c4*(c1*s3 - c3*s1*s2) + s4*(c1*c3 + s1*s2*s3))) - dq4*l5*(c4*(c1*s3 - c3*s1*s2) + s4*(c1*c3 + s1*s2*s3));
dotJ_r_22 = dq1*(l5*(c2*c3*s1*s4 + c2*c4*s1*s3) + l4*c2*s1*s3) - dq3*(l5*(c1*c2*c3*c4 - c1*c2*s3*s4) + l4*c1*c2*c3) + dq2*(l5*(c1*c3*s2*s4 + c1*c4*s2*s3) + l4*c1*s2*s3) - dq4*l5*(c1*c2*c3*c4 - c1*c2*s3*s4);
dotJ_r_23 = - dq2*(l5*(c1*c2*c3*c4 - c1*c2*s3*s4) + l4*c1*c2*c3) - dq1*(l4*(c1*s3 - c3*s1*s2) + l5*(c4*(c1*s3 - c3*s1*s2) + s4*(c1*c3 + s1*s2*s3))) - dq3*(l4*(c3*s1 - c1*s2*s3) + l5*(c4*(c3*s1 - c1*s2*s3) - s4*(s1*s3 + c1*c3*s2))) - dq4*l5*(c4*(c3*s1 - c1*s2*s3) - s4*(s1*s3 + c1*c3*s2));
dotJ_r_24 = - dq2*l5*(c1*c2*c3*c4 - c1*c2*s3*s4) - dq1*l5*(c4*(c1*s3 - c3*s1*s2) + s4*(c1*c3 + s1*s2*s3)) - dq3*l5*(c4*(c3*s1 - c1*s2*s3) - s4*(s1*s3 + c1*c3*s2)) - dq4*l5*(c4*(c3*s1 - c1*s2*s3) - s4*(s1*s3 + c1*c3*s2));

dotJ_r_31 = dq1*(l4*(c1*c3 + s1*s2*s3) + l5*(c4*(c1*c3 + s1*s2*s3) - s4*(c1*s3 - c3*s1*s2)) + l3*c1) - dq2*(l5*(c1*c2*c3*s4 + c1*c2*c4*s3) + l4*c1*c2*s3) - dq3*(l4*(s1*s3 + c1*c3*s2) + l5*(c4*(s1*s3 + c1*c3*s2) + s4*(c3*s1 - c1*s2*s3))) - dq4*l5*(c4*(s1*s3 + c1*c3*s2) + s4*(c3*s1 - c1*s2*s3));
dotJ_r_32 = dq2*(l5*(c3*s1*s2*s4 + c4*s1*s2*s3) + l4*s1*s2*s3) - dq3*(l5*(c2*c3*c4*s1 - c2*s1*s3*s4) + l4*c2*c3*s1) - dq1*(l5*(c1*c2*c3*s4 + c1*c2*c4*s3) + l4*c1*c2*s3) - dq4*l5*(c2*c3*c4*s1 - c2*s1*s3*s4);
dotJ_r_33 = dq3*(l4*(c1*c3 + s1*s2*s3) + l5*(c4*(c1*c3 + s1*s2*s3) - s4*(c1*s3 - c3*s1*s2))) - dq1*(l4*(s1*s3 + c1*c3*s2) + l5*(c4*(s1*s3 + c1*c3*s2) + s4*(c3*s1 - c1*s2*s3))) - dq2*(l5*(c2*c3*c4*s1 - c2*s1*s3*s4) + l4*c2*c3*s1) + dq4*l5*(c4*(c1*c3 + s1*s2*s3) - s4*(c1*s3 - c3*s1*s2));
dotJ_r_34 = dq3*l5*(c4*(c1*c3 + s1*s2*s3) - s4*(c1*s3 - c3*s1*s2)) - dq1*l5*(c4*(s1*s3 + c1*c3*s2) + s4*(c3*s1 - c1*s2*s3)) - dq2*l5*(c2*c3*c4*s1 - c2*s1*s3*s4) + dq4*l5*(c4*(c1*c3 + s1*s2*s3) - s4*(c1*s3 - c3*s1*s2));

dotJ_r_value = [dotJ_r_11, dotJ_r_12, dotJ_r_13, dotJ_r_14;...
                dotJ_r_21, dotJ_r_22, dotJ_r_23, dotJ_r_24;...
                dotJ_r_31, dotJ_r_32, dotJ_r_33, dotJ_r_34];


%左腿雅可比导数

dotJ_l_11 = 0;
dotJ_l_12 = dq7*(l5*(c7*c8*s9 + c7*c9*s8) + l4*c7*s8) - dq8*(l5*(s7*s8*s9 - c8*c9*s7) - l4*c8*s7) - dq9*l5*(s7*s8*s9 - c8*c9*s7);
dotJ_l_13 = dq8*(l5*(c7*c8*s9 + c7*c9*s8) + l4*c7*s8) - dq7*(l5*(s7*s8*s9 - c8*c9*s7) - l4*c8*s7) + dq9*l5*(c7*c8*s9 + c7*c9*s8);
dotJ_l_14 = dq8*l5*(c7*c8*s9 + c7*c9*s8) + dq9*l5*(c7*c8*s9 + c7*c9*s8) - dq7*l5*(s7*s8*s9 - c8*c9*s7);

dotJ_l_21 = dq7*(l5*(c7*c8*s6*s9 + c7*c9*s6*s8) + l4*c7*s6*s8) - dq6*(l4*(c8*s6 - c6*s7*s8) + l5*(c9*(c8*s6 - c6*s7*s8) - s9*(s6*s8 + c6*c8*s7)) + l3*s6) - dq8*(l4*(c6*s8 - c8*s6*s7) + l5*(c9*(c6*s8 - c8*s6*s7) + s9*(c6*c8 + s6*s7*s8))) - dq9*l5*(c9*(c6*s8 - c8*s6*s7) + s9*(c6*c8 + s6*s7*s8));
dotJ_l_22 = dq6*(l5*(c7*c8*s6*s9 + c7*c9*s6*s8) + l4*c7*s6*s8) - dq8*(l5*(c6*c7*c8*c9 - c6*c7*s8*s9) + l4*c6*c7*c8) + dq7*(l5*(c6*c8*s7*s9 + c6*c9*s7*s8) + l4*c6*s7*s8) - dq9*l5*(c6*c7*c8*c9 - c6*c7*s8*s9);
dotJ_l_23 = - dq7*(l5*(c6*c7*c8*c9 - c6*c7*s8*s9) + l4*c6*c7*c8) - dq6*(l4*(c6*s8 - c8*s6*s7) + l5*(c9*(c6*s8 - c8*s6*s7) + s9*(c6*c8 + s6*s7*s8))) - dq8*(l4*(c8*s6 - c6*s7*s8) + l5*(c9*(c8*s6 - c6*s7*s8) - s9*(s6*s8 + c6*c8*s7))) - dq9*l5*(c9*(c8*s6 - c6*s7*s8) - s9*(s6*s8 + c6*c8*s7));
dotJ_l_24 = - dq7*l5*(c6*c7*c8*c9 - c6*c7*s8*s9) - dq6*l5*(c9*(c6*s8 - c8*s6*s7) + s9*(c6*c8 + s6*s7*s8)) - dq8*l5*(c9*(c8*s6 - c6*s7*s8) - s9*(s6*s8 + c6*c8*s7)) - dq9*l5*(c9*(c8*s6 - c6*s7*s8) - s9*(s6*s8 + c6*c8*s7));

dotJ_l_31 = dq6*(l4*(c6*c8 + s6*s7*s8) + l5*(c9*(c6*c8 + s6*s7*s8) - s9*(c6*s8 - c8*s6*s7)) + l3*c6) - dq7*(l5*(c6*c7*c8*s9 + c6*c7*c9*s8) + l4*c6*c7*s8) - dq8*(l4*(s6*s8 + c6*c8*s7) + l5*(c9*(s6*s8 + c6*c8*s7) + s9*(c8*s6 - c6*s7*s8))) - dq9*l5*(c9*(s6*s8 + c6*c8*s7) + s9*(c8*s6 - c6*s7*s8));
dotJ_l_32 = dq7*(l5*(c8*s6*s7*s9 + c9*s6*s7*s8) + l4*s6*s7*s8) - dq8*(l5*(c7*c8*c9*s6 - c7*s6*s8*s9) + l4*c7*c8*s6) - dq6*(l5*(c6*c7*c8*s9 + c6*c7*c9*s8) + l4*c6*c7*s8) - dq9*l5*(c7*c8*c9*s6 - c7*s6*s8*s9);
dotJ_l_33 = dq8*(l4*(c6*c8 + s6*s7*s8) + l5*(c9*(c6*c8 + s6*s7*s8) - s9*(c6*s8 - c8*s6*s7))) - dq6*(l4*(s6*s8 + c6*c8*s7) + l5*(c9*(s6*s8 + c6*c8*s7) + s9*(c8*s6 - c6*s7*s8))) - dq7*(l5*(c7*c8*c9*s6 - c7*s6*s8*s9) + l4*c7*c8*s6) + dq9*l5*(c9*(c6*c8 + s6*s7*s8) - s9*(c6*s8 - c8*s6*s7));
dotJ_l_34 = dq8*l5*(c9*(c6*c8 + s6*s7*s8) - s9*(c6*s8 - c8*s6*s7)) - dq6*l5*(c9*(s6*s8 + c6*c8*s7) + s9*(c8*s6 - c6*s7*s8)) - dq7*l5*(c7*c8*c9*s6 - c7*s6*s8*s9) + dq9*l5*(c9*(c6*c8 + s6*s7*s8) - s9*(c6*s8 - c8*s6*s7));

dotJ_l_value = [dotJ_l_11, dotJ_l_12, dotJ_l_13, dotJ_l_14;...
                dotJ_l_21, dotJ_l_22, dotJ_l_23, dotJ_l_24;...
                dotJ_l_31, dotJ_l_32, dotJ_l_33, dotJ_l_34];

%% 组合

Jw = [J_r_value , zeros(3,4);...
      zeros(3,4), J_l_value];

dot_Jw = [dotJ_r_value , zeros(3,4);...
          zeros(3,4) , dotJ_l_value];

Jw_aug = [J_r_value, zeros(3,1) , zeros(3,5);...
          zeros(3,5), J_l_value, zeros(3,1)];

dot_Jw_aug = [dotJ_r_value, zeros(3,1) , zeros(3,5);...
              zeros(3,5), dotJ_l_value, zeros(3,1)];


