function [g,Q,S,u] = PS_Pexp(d,J,x,dotJ,dotd)
% 用于求解惩罚函数的累加和
%   输入参数：不等式约束系数矩阵和其导数
%   输出参数：惩罚函数的累加和

global sigma;
m = length(d);
r = length(x);

g = zeros(r,1);  Q= zeros(r,r);  S= zeros(r,r);  u= zeros(r,1); 
    for i = 1:m
        
        % yyy = J(i,:)*x;
        % 
        N_i= d(i) - J(i,:)*x;

        g = exp(-sigma*(N_i))*J(i,:)' + g;
        
        Q = exp(-sigma*(N_i))*(J(i,:)'*J(i,:)) + Q;
      
        S = exp(-sigma*(N_i))*(J(i,:)'*dotJ(i,:)) + S;
%         S = exp(-sigma*(N_i))*(J(i,:)'*J(i,:)) + S;       

        u = exp(-sigma*(N_i))*(dotJ(i,:)' - sigma*(J(i,:)')*dotd(i)) + u;
%         u = exp(-sigma*(N_i))*(J(i,:)' - sigma*(J(i,:)')*dotd(i)) + u;
    end
end
