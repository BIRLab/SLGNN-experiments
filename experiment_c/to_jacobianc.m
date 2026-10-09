%此函数用于计算雅可比矩阵的符号运算式

% syms c1 s1 l1 c2 s2 l2 c3 s3 l3 c4 s4 l4 c5 s5 l5 l6; % 定义符号变量
 syms t q1 q2 q3 q4 q5 q6 q7 q8 q9 q10; % 定义关节变量
 syms l1 l2 l3 l4 l5 l6 ;
 syms dq1 dq2 dq3 dq4 dq5 dq6 dq7 dq8 dq9 dq10;

% 右腿---------------------------------------------------------
T01=[  1,    0,   0,    0;...
      0,  cos(q1), -sin(q1),  -l1;...
      0,  sin(q1),  cos(q1),    0;...
      0,    0,   0,   1];

T12=[   cos(q2),  -sin(q2),   0,  l2;...
        sin(q2),   cos(q2),   0,   0;...
        0,    0,   1,   0;...
        0,    0,   0,   1];

T23=[cos(q3),    0,  sin(q3),   0;...
      0,    1,   0,   0;...
    -sin(q3),    0,  cos(q3), -l3;...
      0,    0,   0,   1];    

T34=[cos(q4),   0,  sin(q4),   0;...
      0,   1,   0,   0;...
    -sin(q4),   0,  cos(q4), -l4;...
      0,   0,   0,   1];  

T45=[cos(q5),   0,  sin(q5),   0;...
      0,   1,   0,   0;...
    -sin(q5),   0,  cos(q5), -l5;...
      0,   0,   0,   1];  

T4_position5 = [0; 0; -l5; 1];

toe_p=[l6; 0; 0; 1];

% 左腿---------------------------------------------------------
T06=[  1,    0,   0,   0;...
        0,   cos(q6), -sin(q6),  l1;...
        0,   sin(q6),  cos(q6),   0;...
        0,    0,   0,   1];

T67=[   cos(q7),  -sin(q7),   0,  l2;...
         sin(q7),   cos(q7),   0,   0;...
          0,    0,   1,   0;...
          0,    0,   0,   1];

T78=[cos(q8),    0,  sin(q8),   0;...
        0,    1,   0,   0;...
      -sin(q8),    0,  cos(q8), -l3;...
        0,    0,   0,   1];

T89=[cos(q9),   0,  sin(q9),   0;...
        0,   1,   0,   0;...
      -sin(q9),   0,  cos(q9), -l4;...
        0,   0,   0,   1];

T9_10=[cos(q10),  0,  sin(q10),   0;...
          0,   1,   0,    0;...
        -sin(q10),  0,  cos(q10), -l5;...
          0,   0,   0,   1];

T9_position10 = [0; 0; -l5; 1];

% 符号运算
T04 = T01 * T12 * T23 * T34 * T4_position5;%* T45; * T56;
T09 = T06 * T67 * T78 * T89 * T9_position10;

position_r = T04(1:3, 1); % 提取末端执行器的位置向量
position_l = T09(1:3, 1);

variables = [q1, q2, q3, q4]; % 定义关节变量向量
J_r = jacobian(position_r, [q1, q2, q3, q4]) % 计算雅可比矩阵
J_l = jacobian(position_l, [q6, q7, q8, q9])
% % 指定关节角度值
% q1_val = 0; q2_val = 1; q3_val = 0.6; q4_val = 1.1; q5_val = 0.3;
% 
% % 替换符号变量为具体的数值
% J_numeric = subs(J, [q1, q2, q3, q4, q5], [q1_val, q2_val, q3_val, q4_val, q5_val]);
% 
% disp("给定关节角度的雅可比矩阵：");
% disp(J_numeric);

% % 计算雅可比矩阵的导数
% syms dq1 dq2 dq3 dq4 dq5; % 定义关节速度符号变量
% q_dot = [dq1; dq2; dq3; dq4]; % 关节速度向量
% J_dot = simplify(jacobian(J * q_dot, [q1, q2, q3, q4]) * q_dot);


dotJ_r = diff(J_r, q1) * dq1 + diff(J_r, q2) * dq2 + diff(J_r, q3) * dq3 + diff(J_r, q4) * dq4;
dotJ_l = diff(J_l, q6) * dq6 + diff(J_l, q7) * dq7 + diff(J_l, q8) * dq8 + diff(J_l, q9) * dq9;

% 显示结果
% disp("雅可比矩阵：");
% disp(J_l);
% disp(J_r);
% disp("雅可比矩阵的导数：");
% disp(dotJ_r);
% disp(dotJ_l);
%%  雅可比矩阵计算

%右腿雅可比
J_r_11 = 0;
J_r_12 = l5*(cos(q3)*sin(q2)*sin(q4) + cos(q4)*sin(q2)*sin(q3)) + l4*sin(q2)*sin(q3);
J_r_13 = -l5*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4)) - l4*cos(q2)*cos(q3);
J_r_14 = -l5*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4));

J_r_21 = l4*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) + l5*(cos(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2))) + l3*cos(q1);
J_r_22 = - l5*(cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*cos(q2)*cos(q4)*sin(q3)) - l4*cos(q1)*cos(q2)*sin(q3);
J_r_23 = - l4*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - l5*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)));
J_r_24 = -l5*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)));

J_r_31 = l4*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)) + l5*(cos(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + l3*sin(q1);
J_r_32 = - l5*(cos(q2)*cos(q3)*sin(q1)*sin(q4) + cos(q2)*cos(q4)*sin(q1)*sin(q3)) - l4*cos(q2)*sin(q1)*sin(q3);
J_r_33 = l4*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)) + l5*(cos(q4)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)));
J_r_34 = l5*(cos(q4)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)));
J_r_value = [J_r_11, J_r_12, J_r_13, J_r_14;...
             J_r_21, J_r_22, J_r_23, J_r_24;...
             J_r_31, J_r_32, J_r_33, J_r_34];


%左腿雅可比

J_l_11 = 0;
J_l_12 = l5*(cos(q8)*sin(q7)*sin(q9) + cos(q9)*sin(q7)*sin(q8)) + l4*sin(q7)*sin(q8);
J_l_13 = - l5*(cos(q7)*cos(q8)*cos(q9) - cos(q7)*sin(q8)*sin(q9)) - l4*cos(q7)*cos(q8);
J_l_14 = -l5*(cos(q7)*cos(q8)*cos(q9) - cos(q7)*sin(q8)*sin(q9));

J_l_21 = l4*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)) + l5*(cos(q9)*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)) - sin(q9)*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7))) + l3*cos(q6);
J_l_22 = - l5*(cos(q6)*cos(q7)*cos(q8)*sin(q9) + cos(q6)*cos(q7)*cos(q9)*sin(q8)) - l4*cos(q6)*cos(q7)*sin(q8);
J_l_23 = - l4*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7)) - l5*(cos(q9)*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7)) + sin(q9)*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8)));
J_l_24 = -l5*(cos(q9)*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7)) + sin(q9)*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8))); 

J_l_31 = l4*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8)) + l5*(cos(q9)*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8)) - sin(q9)*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7))) + l3*sin(q6);
J_l_32 = - l5*(cos(q7)*cos(q8)*sin(q6)*sin(q9) + cos(q7)*cos(q9)*sin(q6)*sin(q8)) - l4*cos(q7)*sin(q6)*sin(q8);
J_l_33 = l4*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7)) + l5*(cos(q9)*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7)) + sin(q9)*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)));
J_l_34 =  l5*(cos(q9)*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7)) + sin(q9)*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)));

J_l_value = [J_l_11, J_l_12, J_l_13, J_l_14;...
             J_l_21, J_l_22, J_l_23, J_l_24;...
             J_l_31, J_l_32, J_l_33, J_l_34];

%%  计算结果
% %% 计算雅可比矩阵导数
% 
% %右腿雅可比导数
% dotJ_r_11 = 0;
% dotJ_r_12 = dq2*(l5*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3)) + l4*cos(q2)*sin(q3)) - dq3*(l5*(sin(q2)*sin(q3)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - l4*cos(q3)*sin(q2)) - dq4*l5*(sin(q2)*sin(q3)*sin(q4) - cos(q3)*cos(q4)*sin(q2));
% dotJ_r_13 = dq3*(l5*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3)) + l4*cos(q2)*sin(q3)) - dq2*(l5*(sin(q2)*sin(q3)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - l4*cos(q3)*sin(q2)) + dq4*l5*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3));
% dotJ_r_14 = dq3*l5*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3)) + dq4*l5*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3)) - dq2*l5*(sin(q2)*sin(q3)*sin(q4) - cos(q3)*cos(q4)*sin(q2));
% 
% dotJ_r_21 = dq2*(l5*(cos(q2)*cos(q3)*sin(q1)*sin(q4) + cos(q2)*cos(q4)*sin(q1)*sin(q3)) + l4*cos(q2)*sin(q1)*sin(q3)) - dq1*(l4*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)) + l5*(cos(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + l3*sin(q1)) - dq3*(l4*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)) + l5*(cos(q4)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)))) - dq4*l5*(cos(q4)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)));
% dotJ_r_22 = dq1*(l5*(cos(q2)*cos(q3)*sin(q1)*sin(q4) + cos(q2)*cos(q4)*sin(q1)*sin(q3)) + l4*cos(q2)*sin(q1)*sin(q3)) - dq3*(l5*(cos(q1)*cos(q2)*cos(q3)*cos(q4) - cos(q1)*cos(q2)*sin(q3)*sin(q4)) + l4*cos(q1)*cos(q2)*cos(q3)) + dq2*(l5*(cos(q1)*cos(q3)*sin(q2)*sin(q4) + cos(q1)*cos(q4)*sin(q2)*sin(q3)) + l4*cos(q1)*sin(q2)*sin(q3)) - dq4*l5*(cos(q1)*cos(q2)*cos(q3)*cos(q4) - cos(q1)*cos(q2)*sin(q3)*sin(q4));
% dotJ_r_23 = - dq2*(l5*(cos(q1)*cos(q2)*cos(q3)*cos(q4) - cos(q1)*cos(q2)*sin(q3)*sin(q4)) + l4*cos(q1)*cos(q2)*cos(q3)) - dq1*(l4*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)) + l5*(cos(q4)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)))) - dq3*(l4*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)) + l5*(cos(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) - dq4*l5*(cos(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)));
% dotJ_r_24 = - dq2*l5*(cos(q1)*cos(q2)*cos(q3)*cos(q4) - cos(q1)*cos(q2)*sin(q3)*sin(q4)) - dq1*l5*(cos(q4)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3))) - dq3*l5*(cos(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) - dq4*l5*(cos(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2))); 
% 
% dotJ_r_31 = dq1*(l4*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) + l5*(cos(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2))) + l3*cos(q1)) - dq2*(l5*(cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*cos(q2)*cos(q4)*sin(q3)) + l4*cos(q1)*cos(q2)*sin(q3)) - dq3*(l4*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + l5*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)))) - dq4*l5*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)));
% dotJ_r_32 = dq2*(l5*(cos(q3)*sin(q1)*sin(q2)*sin(q4) + cos(q4)*sin(q1)*sin(q2)*sin(q3)) + l4*sin(q1)*sin(q2)*sin(q3)) - dq3*(l5*(cos(q2)*cos(q3)*cos(q4)*sin(q1) - cos(q2)*sin(q1)*sin(q3)*sin(q4)) + l4*cos(q2)*cos(q3)*sin(q1)) - dq1*(l5*(cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*cos(q2)*cos(q4)*sin(q3)) + l4*cos(q1)*cos(q2)*sin(q3)) - dq4*l5*(cos(q2)*cos(q3)*cos(q4)*sin(q1) - cos(q2)*sin(q1)*sin(q3)*sin(q4));
% dotJ_r_33 = dq3*(l4*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) + l5*(cos(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)))) - dq1*(l4*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + l5*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)))) - dq2*(l5*(cos(q2)*cos(q3)*cos(q4)*sin(q1) - cos(q2)*sin(q1)*sin(q3)*sin(q4)) + l4*cos(q2)*cos(q3)*sin(q1)) + dq4*l5*(cos(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)));
% dotJ_r_34 = dq3*l5*(cos(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2))) - dq1*l5*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3))) - dq2*l5*(cos(q2)*cos(q3)*cos(q4)*sin(q1) - cos(q2)*sin(q1)*sin(q3)*sin(q4)) + dq4*l5*(cos(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)));
% 
% dotJ_r_value = [dotJ_r_11, dotJ_r_12, dotJ_r_13, dotJ_r_14;...
%                 dotJ_r_21, dotJ_r_22, dotJ_r_23, dotJ_r_24;...
%                 dotJ_r_31, dotJ_r_32, dotJ_r_33, dotJ_r_34];
% 
% 
% %左腿雅可比导数
% 
% dotJ_l_11 = 0;
% dotJ_l_12 = dq7*(l5*(c7*c8*s9 + c7*c9*s8) + l4*c7*s8) - dq8*(l5*(s7*s8*s9 - c8*c9*s7) - l4*c8*s7) - dq9*l5*(s7*s8*s9 - c8*c9*s7);
% dotJ_l_13 = dq8*(l5*(c7*c8*s9 + c7*c9*s8) + l4*c7*s8) - dq7*(l5*(s7*s8*s9 - c8*c9*s7) - l4*c8*s7) + dq9*l5*(c7*c8*s9 + c7*c9*s8);
% dotJ_l_14 = dq8*l5*(c7*c8*s9 + c7*c9*s8) + dq9*l5*(c7*c8*s9 + c7*c9*s8) - dq7*l5*(s7*s8*s9 - c8*c9*s7);
% 
% dotJ_l_21 = dq7*(l5*(cos(q7)*cos(q8)*sin(q6)*sin(q9) + cos(q7)*cos(q9)*sin(q6)*sin(q8)) + l4*cos(q7)*sin(q6)*sin(q8)) - dq6*(l4*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8)) + l5*(cos(q9)*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8)) - sin(q9)*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7))) + l3*sin(q6)) - dq8*(l4*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7)) + l5*(cos(q9)*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7)) + sin(q9)*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)))) - dq9*l5*(cos(q9)*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7)) + sin(q9)*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)));
% dotJ_l_22 = dq6*(l5*(cos(q7)*cos(q8)*sin(q6)*sin(q9) + cos(q7)*cos(q9)*sin(q6)*sin(q8)) + l4*cos(q7)*sin(q6)*sin(q8)) - dq8*(l5*(cos(q6)*cos(q7)*cos(q8)*cos(q9) - cos(q6)*cos(q7)*sin(q8)*sin(q9)) + l4*cos(q6)*cos(q7)*cos(q8)) + dq7*(l5*(cos(q6)*cos(q8)*sin(q7)*sin(q9) + cos(q6)*cos(q9)*sin(q7)*sin(q8)) + l4*cos(q6)*sin(q7)*sin(q8)) - dq9*l5*(cos(q6)*cos(q7)*cos(q8)*cos(q9) - cos(q6)*cos(q7)*sin(q8)*sin(q9));
% dotJ_l_23 = - dq7*(l5*(cos(q6)*cos(q7)*cos(q8)*cos(q9) - cos(q6)*cos(q7)*sin(q8)*sin(q9)) + l4*cos(q6)*cos(q7)*cos(q8)) - dq6*(l4*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7)) + l5*(cos(q9)*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7)) + sin(q9)*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)))) - dq8*(l4*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8)) + l5*(cos(q9)*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8)) - sin(q9)*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7)))) - dq9*l5*(cos(q9)*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8)) - sin(q9)*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7)));
% dotJ_l_24 = - dq7*l5*(cos(q6)*cos(q7)*cos(q8)*cos(q9) - cos(q6)*cos(q7)*sin(q8)*sin(q9)) - dq6*l5*(cos(q9)*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7)) + sin(q9)*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8))) - dq8*l5*(cos(q9)*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8)) - sin(q9)*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7))) - dq9*l5*(cos(q9)*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8)) - sin(q9)*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7)));
% 
% dotJ_l_31 = dq6*(l4*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)) + l5*(cos(q9)*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)) - sin(q9)*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7))) + l3*cos(q6)) - dq7*(l5*(cos(q6)*cos(q7)*cos(q8)*sin(q9) + cos(q6)*cos(q7)*cos(q9)*sin(q8)) + l4*cos(q6)*cos(q7)*sin(q8)) - dq8*(l4*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7)) + l5*(cos(q9)*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7)) + sin(q9)*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8)))) - dq9*l5*(cos(q9)*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7)) + sin(q9)*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8)));
% dotJ_l_32 = dq7*(l5*(cos(q8)*sin(q6)*sin(q7)*sin(q9) + cos(q9)*sin(q6)*sin(q7)*sin(q8)) + l4*sin(q6)*sin(q7)*sin(q8)) - dq8*(l5*(cos(q7)*cos(q8)*cos(q9)*sin(q6) - cos(q7)*sin(q6)*sin(q8)*sin(q9)) + l4*cos(q7)*cos(q8)*sin(q6)) - dq6*(l5*(cos(q6)*cos(q7)*cos(q8)*sin(q9) + cos(q6)*cos(q7)*cos(q9)*sin(q8)) + l4*cos(q6)*cos(q7)*sin(q8)) - dq9*l5*(cos(q7)*cos(q8)*cos(q9)*sin(q6) - cos(q7)*sin(q6)*sin(q8)*sin(q9));
% dotJ_l_33 = dq8*(l4*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)) + l5*(cos(q9)*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)) - sin(q9)*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7)))) - dq6*(l4*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7)) + l5*(cos(q9)*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7)) + sin(q9)*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8)))) - dq7*(l5*(cos(q7)*cos(q8)*cos(q9)*sin(q6) - cos(q7)*sin(q6)*sin(q8)*sin(q9)) + l4*cos(q7)*cos(q8)*sin(q6)) + dq9*l5*(cos(q9)*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)) - sin(q9)*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7)));
% dotJ_l_34 = dq8*l5*(cos(q9)*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)) - sin(q9)*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7))) - dq6*l5*(cos(q9)*(sin(q6)*sin(q8) + cos(q6)*cos(q8)*sin(q7)) + sin(q9)*(cos(q8)*sin(q6) - cos(q6)*sin(q7)*sin(q8))) - dq7*l5*(cos(q7)*cos(q8)*cos(q9)*sin(q6) - cos(q7)*sin(q6)*sin(q8)*sin(q9)) + dq9*l5*(cos(q9)*(cos(q6)*cos(q8) + sin(q6)*sin(q7)*sin(q8)) - sin(q9)*(cos(q6)*sin(q8) - cos(q8)*sin(q6)*sin(q7)));
% 
% dotJ_l_value = [dotJ_l_11, dotJ_l_12, dotJ_l_13, dotJ_l_14;...
%                 dotJ_l_21, dotJ_l_22, dotJ_l_23, dotJ_l_24;...
%                 dotJ_l_31, dotJ_l_32, dotJ_l_33, dotJ_l_34];
