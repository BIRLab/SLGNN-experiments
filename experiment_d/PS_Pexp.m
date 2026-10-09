function [g,Q,S,u] = PS_Pexp(d,J,x,dotJ,dotd,penalty_sigma)
% 用于求解惩罚函数的累加和
%   输入参数：不等式约束系数矩阵和其导数
%   输出参数：惩罚函数的累加和

% 基于实验 a 的原函数；可显式传入 sigma，避免不同实验的全局值互相影响。
if nargin < 6
    global sigma;
    penalty_sigma = sigma;
end
m = length(d);
r = length(x);

g = zeros(r,1);  Q= zeros(r,r);  S= zeros(r,r);  u= zeros(r,1); 
    for i = 1:m
        
        % yyy = J(i,:)*x;
        % 
        N_i= d(i) - J(i,:)*x;

        g = exp(-penalty_sigma*(N_i))*J(i,:)' + g;
        
        Q = exp(-penalty_sigma*(N_i))*(J(i,:)'*J(i,:)) + Q;
      
        S = exp(-penalty_sigma*(N_i))*(J(i,:)'*dotJ(i,:)) + S;
%         S = exp(-penalty_sigma*(N_i))*(J(i,:)'*J(i,:)) + S;       

        u = exp(-penalty_sigma*(N_i))*(dotJ(i,:)' - penalty_sigma*(J(i,:)')*dotd(i)) + u;
%         u = exp(-penalty_sigma*(N_i))*(J(i,:)' - penalty_sigma*(J(i,:)')*dotd(i)) + u;
    end
end
