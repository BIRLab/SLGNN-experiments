# Source data / 论文配图数据

Archived data for the SLGNN bipedal motion-generation manuscript, organized by
the figure numbering in `latex_v11_ztm/00-main.tex` on 16 October 2026.
These exports use saved results and figure arrays; the simulations were not rerun.

本目录按 2026-10-16 稿件的配图编号整理 SLGNN 双足机器人运动生成实验数据。
数据直接来自已保存的实验结果和 MATLAB 图形数组，没有以重新运行结果替换论文数据。

## Contents / 内容

| Folder or file / 文件 | Content / 内容 |
| --- | --- |
| `fig4/` | Straight walking: saved geometry, feasible-region scan, slack rates and eight physical joint rates / 直线实验 |
| `fig5/` | All 15 residual traces, pointwise medians, trial metrics, paired initial states and solver settings / 三种求解器对比 |
| `fig6/` | Stair ascent: saved geometry, both knee rates, normalized speed, slack rates and horizontal errors / 爬楼梯实验 |
| `tables/` | Tables 1--3, with available full-precision values / 表格数据 |
| `native/` | Original FIG/MAT/CSV archives, three EPS-only panels, published panel PDFs and Fig.7 photographs / 原生来源与配图 |
| `figure_manifest.csv` | Panel-by-panel mapping and availability status / 面板索引与完整性 |
| `data_dictionary.csv` | Column meanings and units / 数据字典 |
| `simulation_joint_mapping.csv` | Array slot to physical joint mapping and simulation bounds / 关节映射 |
| `provenance.json` | Source filenames, transformations and SHA-256 hashes / 来源记录 |
| `SHA256SUMS.txt` | Checksums of all release files / 完整性校验 |

CSV files are UTF-8, comma-delimited, with a header row. MATLAB numerical-array
exports use 17 significant decimal digits. Empty cells in Table2 mean values
omitted in the manuscript, not zero. `NaN` threshold time in Table3 means the
threshold was not reached within 10 s. The native archives are retained so these
exports can be independently inspected.

CSV 为 UTF-8 编码、逗号分隔，首行为列名。MATLAB 数组以 17 位有效数字导出。
表2空单元格表示原稿未报告该值；表3的 `NaN` 表示 10 秒内未达到阈值，均不表示零。

## Availability limits / 数据完整性说明

- **Fig.4b, 4g and 4h:** only the original EPS artwork and the published PDF
  panels were located. Their original numerical simulation arrays are unavailable
  in this archive. No digitized or rerun values are presented as original data.
  In particular, Fig.4b was not silently reconstructed from Fig.4e/f.
- **Fig.4a and Fig.6a:** exported coordinates are the geometry and foot trajectories
  stored in the saved MATLAB figures. They are not full timed joint-state logs.
- **Fig.7:** the published photographic sequence is included. Individual original
  camera files and numerical hardware logs were not located and are not included.
- **Figs.1--3:** diagrams and artwork, without experimental numerical time series.

- **Fig.4b、4g、4h：**目前只有原 EPS 和论文 PDF 面板，未找到原始仿真数值数组。
  未将图像数字化或重跑的数据冒充原始数据，也未用 Fig.4e/f 反推替换 Fig.4b。
- **Fig.4a、Fig.6a：**坐标来自已存 FIG 的几何图形与足端轨迹，不是完整带时间戳的关节日志。
- **Fig.7：**提供论文照片拼图；未找到原始单张相机文件及硬件数值日志。
- **Fig.1--3：**为示意图，不对应数值时间序列。

## Conventions / 约定

Physical joint numbering follows manuscript Fig.2: q0 is the waist, q4 is the
right knee, and q9 is the left knee. Eight optimized joints are q1--q4 and
q6--q9. Historical scan `critical_joint_id=8` means **array slot 8**, hence
physical q9; see `simulation_joint_mapping.csv`. Historical native method names
`smg-RNN` and `SP-VP-RNN` correspond to SLGNN; the archives remain unchanged.
Slack curves are velocities in mm/s. Horizontal error curves are velocity-level
errors in m/s, not position errors. Geometry uses mm; joint rates use rad/s.

The feasible-speed scan (Fig.4c/Table2) used p=0 for MVN and p=200 for SLGNN.
This scan must be distinguished from the p=100 straight/stair examples and
the three-solver comparison. Table1 contains hardware specifications; simulation
bounds are listed separately. Velocity feasibility alone does not establish
foot-clearance feasibility or dynamic stability.

关节编号以 Fig.2 为准：q0 为腰关节，q4/q9 分别为右/左膝关节。
原扫描文件中的关节 8 指八维数组第 8 项，对应物理 q9；原生文件保留历史标记。
松弛变量曲线单位为 mm/s，水平误差为 m/s，几何坐标为 mm，关节速度为 rad/s。
Fig.4c/表2扫描中 SLGNN 的 p=200，与 p=100 的直线、爬楼梯和求解器对比实验区分。
硬件参数和仿真约束分列，速度可行不等同于足部间隙或动力学稳定性得到验证。

## Read and reproduce / 读取与复核

Requires base MATLAB; scripts tested with MATLAB R2026a on Windows.
From the repository root:

需要基础 MATLAB，已在 Windows MATLAB R2026a 验证。在仓库根目录运行：

```matlab
addpath('source_data/scripts')
replot_source_data       % Inspect saved numeric curves / 查看已存数值曲线
export_source_data      % Regenerate CSVs from archives / 从原生来源重导出 CSV
```

`export_source_data` checks numeric-array CSV round trips, error norms, paired
initial states and Fig.5 statistics. Late residual RMS is
`sqrt(trapz(t(t>=1), residual(t>=1).^2)/9)`, followed by the median over five
paired trials. Pointwise median curves are computed separately. Table3 retains
the original paper values (GNN 1.31e-1, ZNN 4.88e-6, SLGNN 6.16e-7 after rounding).
Rerunning `experiment_d/main.m` can yield small numerical differences with
MATLAB version and solver behavior; it does not overwrite this archive.

导出脚本校验 CSV 数值往返、误差范数、配对初值和 Fig.5 的统计量。
晚期残差先对每组试验做 1--10 s 时间加权 RMS，再取五组中位数；
逐时刻中位曲线另行计算。原论文数值与后来重跑结果分别保存。

Code and data in this repository use the existing BSD-3-Clause license;
see `../LICENSE`. Cite the associated SLGNN paper when available and identify
the Git commit used for the data. A GitHub URL is not a DOI.

代码与数据沿用仓库 BSD-3-Clause 协议。引用时请注明对应论文及所使用的 Git 提交；
GitHub 地址不是 DOI。
