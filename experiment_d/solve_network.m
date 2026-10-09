function run = solve_network(method, P, cfg, y0)
%SOLVE_NETWORK 与实验 a、b 一样使用 ode15s 求解 bepid_VP_RNN。
count = 0;
options = odeset('RelTol', cfg.RelTol, 'AbsTol', cfg.AbsTol, 'MaxStep', cfg.MaxStep);
if strcmp(method, 'GNN')
    options = odeset(options, 'Jacobian', @gnn_jac);
end
clock = tic;
sol = ode15s(@rhs, [0 cfg.T], y0, options);
elapsed = toc(clock);
assert(abs(sol.x(end)-cfg.T) < 1e-9, 'Integration incomplete');
tt = cfg.t(cfg.t <= cfg.T);
run = struct('y', deval(sol, tt)', 't', tt, 'method', method, ...
    'wall', elapsed, 'calls', count, 'steps', numel(sol.x)-1);

    function dy = rhs(t, y)
        count = count + 1;
        dy = bepid_VP_RNN(t, y, method, P, cfg);
    end
    function J = gnn_jac(t, y)
        [~, J] = bepid_VP_RNN(t, y, method, P, cfg);
    end
end
