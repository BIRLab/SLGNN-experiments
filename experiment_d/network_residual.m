function [F, D, Ft, third] = network_residual(t, y, P, cfg)
%NETWORK_RESIDUAL 基于实验 a 的 PS_Pexp 构建统一缩放 KKT 残差。
% y=[z(10);mu(6)], x=S*z；三种网络共享本函数，保持相同问题。
[A, dA, c, dc] = network_problem(t, P, cfg);
z = y(1:10);
mu = y(11:16);
QP_x = cfg.S * z;
QP_J = [eye(10); -eye(10)];
QP_d = [cfg.limits; cfg.limits];
[PS_Pexp_g, PS_Pexp_Q] = PS_Pexp(QP_d, QP_J, QP_x, ...
    zeros(20, 10), zeros(20, 1), cfg.sigma);
g = cfg.p * cfg.sigma * cfg.S * PS_Pexp_g / cfg.objective_scale;
h = cfg.p * cfg.sigma^2 * cfg.S * PS_Pexp_Q * cfg.S / cfg.objective_scale;
F = [cfg.Hs*z + g + A'*mu; A*z - c];
D = [cfg.Hs + h, A'; A, zeros(6)];
Ft = [dA'*mu; dA*z - dc];
% GNN 的解析 Jacobian 需要惩罚的三阶导数。
third = cfg.p * cfg.sigma^3 / cfg.objective_scale * cfg.limits.^3 .* PS_Pexp_g;
end
