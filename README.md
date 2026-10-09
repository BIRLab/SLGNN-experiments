# SLGNN Experiments

## 中文

SLGNN（Spinal Locomotor Generation Neural Network）的 MATLAB 实验代码，包括直线行走、楼梯行走、速度可行域扫描，以及与 GNN、ZNN 的比较。

- 环境：MATLAB R2026a（已在 Windows 上验证）。
- 依赖：主实验仅需基础 MATLAB，无需 Python 或 Optimization Toolbox。可选的 `to_jacobianc.m` 符号推导需要 Symbolic Math Toolbox。
- 运行：进入任一实验文件夹，在 MATLAB 中运行 `main`，即可计算并显示结果。各实验请分别运行，避免同名函数冲突。
- 实验 c 附带扫描数据；重新扫描可运行 `run_speed_limit_scan_mvn_baseline`。实验 d 的共同输入数据已包含，三种网络使用同一输入进行比较。

## English

MATLAB experiments for SLGNN (Spinal Locomotor Generation Neural Network): straight walking, stair climbing, feasible-speed scanning, and comparison with GNN and ZNN.

- Environment: MATLAB R2026a, tested on Windows.
- Dependencies: base MATLAB for the main experiments; no Python or Optimization Toolbox required. Optional symbolic derivation with `to_jacobianc.m` requires Symbolic Math Toolbox.
- Run: open an experiment folder in MATLAB and run `main` to compute and display the results. Run each experiment separately to avoid conflicts between functions with the same names.
- Experiment c includes scan data; run `run_speed_limit_scan_mvn_baseline` to repeat the scan. Experiment d includes the shared replay input used by all three networks.

| Folder / 文件夹 | Experiment / 实验 |
| --- | --- |
| `experiment_a` | Straight walking / 直线行走 |
| `experiment_b` | Stair climbing / 楼梯行走 |
| `experiment_c` | Feasible-speed scan / 速度可行域扫描 |
| `experiment_d` | SLGNN vs. GNN and ZNN / 三网络比较 |
