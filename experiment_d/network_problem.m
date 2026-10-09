function [A, dA, c, dc] = network_problem(t, P, cfg)
%NETWORK_PROBLEM 与实验 a 相同的机器人雅可比及垂向松弛映射。
q = ppval(P.q, t);
dotq = ppval(P.dq, t);
[~, ~, Jw_aug, dot_Jw_aug] = ztm_jacobian(q, dotq);
A = (Jw_aug - cfg.slack) * cfg.S / cfg.velocity_scale;
dA = dot_Jw_aug * cfg.S / cfg.velocity_scale;
c = ppval(P.c, t) / cfg.velocity_scale;
dc = ppval(P.dc, t) / cfg.velocity_scale;
end
