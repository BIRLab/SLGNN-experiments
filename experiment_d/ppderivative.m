function dp = ppderivative(pp)
%PPDERIVATIVE 分段多项式的解析导数，不依赖额外工具箱。
[br, co, ~, k, d] = unmkpp(pp);
dp = mkpp(br, co(:, 1:k-1) .* (k-1:-1:1), d);
end
