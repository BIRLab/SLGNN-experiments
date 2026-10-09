function api = run_network_comparison(mode)
%RUN_NETWORK_COMPARISON 兼容旧入口；主要实验流程现位于 main.m。
if nargin == 0, mode = 'full'; end
if nargout > 0
    api = main(mode);
else
    main(mode);
end
end
