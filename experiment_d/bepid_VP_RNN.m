function [doty, J] = bepid_VP_RNN(t, y, method, P, cfg)
%BEPID_VP_RNN 基于实验 a 的神经动力学入口，扩展为三种求解器。
% a 中同时积分姿态和网络状态；d 共享回放姿态，仅积分16维网络状态。
% 完整 KKT 导数及共同变量缩放沿用实验 d，避免改变已发表比较定义。
[F, RNN_Q, Ft, third] = network_residual(t, y, P, cfg);
switch method
    case 'GNN'
        doty = -cfg.gamma * (RNN_Q' * F);
    case 'ZNN'
        doty = RNN_Q \ (-Ft - cfg.gamma * F);
    case {'SLGNN', 'smg-RNN'} % 旧名称保留给已有调参脚本
        beta = 1;
        if isfield(cfg, 'beta'), beta = cfg.beta; end
        doty = RNN_Q \ (-Ft - cfg.gamma * exp(beta*t) * F);
    otherwise
        error('bepid_VP_RNN:UnknownMethod', 'Unknown network: %s', method);
end
if nargout > 1
    assert(strcmp(method, 'GNN'), 'Only GNN uses the explicit ODE Jacobian.');
    correction = zeros(16);
    correction(1:10, 1:10) = diag(third .* F(1:10));
    J = -cfg.gamma * (RNN_Q' * RNN_Q + correction);
end
end
