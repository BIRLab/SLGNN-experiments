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

## 数据与可用性 / Source Data and Availability

论文配图对应的数据、单位、来源及完整性说明见 [source_data](source_data/README.md)。
提供 CSV、原生图形/结果及导出、查看脚本。Fig.4b、4g、4h 仅保留矢量图，
未找到原始数值数组；Fig.7 提供论文照片拼图，不包含硬件数值日志。

Data organized by manuscript figure, units, provenance and availability limits
are documented in [source_data](source_data/README.md), with CSV exports,
native archives and export/visualization scripts. Fig.4b, 4g and 4h have
vector figure files only; their original numerical arrays are unavailable.
Fig.7 includes the published photographic montage, without numerical hardware logs.

数据与代码可用性声明 / [Data and code availability statements](source_data/DATA_AVAILABILITY.md).

## 开源协议 / License

本项目采用 BSD-3-Clause 协议，详见 LICENSE 文件。

This project is licensed under the BSD-3-Clause License.
See the LICENSE file for details.

## 引用 / Citation

如果本代码对您的研究有帮助，请引用相关论文。
完整引用信息将在论文发表后补充。

If this code is useful for your research, please cite the associated paper.
Full citation details will be added upon publication.
