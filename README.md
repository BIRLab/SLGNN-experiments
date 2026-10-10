# SLGNN

SLGNN 实验代码，应用于求解双足机器人运动生成问题。

Experimental code for SLGNN, applied to motion generation for bipedal robots.

## 实验 / Experiments

| 文件夹 / Folder | 内容 / Description |
| --- | --- |
| `experiment_a` | 直线行走 / Straight walking |
| `experiment_b` | 楼梯行走 / Stair climbing |
| `experiment_c` | 速度可行域扫描 / Feasible-speed scanning |
| `experiment_d` | SLGNN、GNN 与 ZNN 对比 / Comparison of SLGNN, GNN and ZNN |

## 环境与依赖 / Environment and Dependencies

- MATLAB R2026a，已在 Windows 上验证。
- 主实验仅需基础 MATLAB；可选的符号推导脚本
  `to_jacobianc.m` 需要 Symbolic Math Toolbox。

- MATLAB R2026a, tested on Windows.
- The main experiments require only base MATLAB. The optional symbolic
  derivation script `to_jacobianc.m` requires Symbolic Math Toolbox.

## 运行 / Usage

在 MATLAB 中进入所选实验文件夹，运行 `main`。
必要输入数据已包含，各实验请分别运行。

Open the selected experiment folder in MATLAB and run `main`.
Required input data are included. Run each experiment separately.

```matlab
cd experiment_a
main
```

## 开源协议 / License

本项目采用 BSD-3-Clause 协议，详见 LICENSE 文件。

This project is licensed under the BSD-3-Clause License.
See the LICENSE file for details.

## 引用 / Citation

如果本代码对您的研究有帮助，请引用相关论文。
完整引用信息将在论文发表后补充。

If this code is useful for your research, please cite the associated paper.
Full citation details will be added upon publication.
