# 基于卡尔曼滤波与因子图相结合的协同导航方法

本仓库是一个 MATLAB 协同导航研究工程。当前主要实验将 SINS/Kalman
滤波、因子图优化、滑动窗口、在线 CUSUM 测距边加权与可选的 IMU 预积分
结合，用于比较不同的鲁棒协同定位策略。

## 当前能力

- 单历元 Equal-FGO 与 CUSUM-FGO 对比；
- 固定长度滑动窗口，以及可配置的告警测距边准入策略；
- 3F3L（三架从机、三架领机）实验配置；
- 可选的 15 状态 IMU 预积分滑动窗口因子；
- 单元测试、烟雾测试和实验实现报告。

实验结果依赖于配置、随机种子和故障场景；仓库中的结果不应被解读为对
所有场景都成立的精度结论。

## 快速开始

在 MATLAB 中切换到本仓库根目录后，先运行与目标路径相符的快速测试：

```matlab
test_stage1_3f3l_smoke
test_stage1_3f3l_imu_preint_smoke
```

运行当前 3F3L 的 IMU 预积分对比：

```matlab
cfg = stage1_cusum_3f3l_config('full');
cfg.seeds = 23;
cfg.fault_enable = true;
cfg.fault_edge = [2, 4];
cfg.fault_start = 100;
cfg.fault_end = 150;
cfg.fault_bias = 3;
cfg.imu_preint_window_length = 3;
cfg.plot_component_comparison = false;
report = run_stage1_imu_preintegration_comparison(cfg);
```

常规的 CUSUM 对比入口为 `run_stage1_cusum_comparison.m`；默认和 3F3L
配置分别见 `stage1_cusum_default_config.m` 与
`stage1_cusum_3f3l_config.m`。

## 重要文件

- `factor_graph_sliding_window.m`：位置状态滑动窗口因子图；
- `factor_graph_sliding_window_imu_preint.m`：独立的 15 状态 IMU 预积分
  因子图；
- `compute_signed_cusum_edge_weights.m`：在线 CUSUM 测距边权重；
- `IMU预积分_实施与验证报告.md`：预积分实现、验证与已知限制；
- `使用说明_第一阶段_CUSUM.md`：完整实验入口和配置说明；
- `AGENTS.md`：供 Codex 使用的代码、测试和 GitHub 协作规范。

## GitHub 与代码审查

`main` 是稳定协作基线；每个修改应从 `main` 创建聚焦的分支，并通过
提交记录和 Pull Request 说明变更。仓库规则要求 Codex 在完成一组已验证
的逻辑修改后，将该分支推送到 GitHub。

在网页版 ChatGPT 连接本仓库后，可以使用下面的审查提示：

```text
请审查仓库 excalibur-saber-666/zhu 的 <分支名> 分支；只提出建议，不直接修改。
重点检查：MATLAB 维度/索引错误、ENU 与单位一致性、因子残差和协方差白化、
CUSUM 是否错误使用真值或故障标签、随机性可复现性，以及测试覆盖。
按 P0–P3 排序，每项给出 文件:行号、问题、依据、建议补丁和应运行的测试。
```

收到审查意见后，将意见或 Issue/PR 链接交给 Codex。Codex 会核实建议、
完成必要修改和测试，并把变更、验证结果及提交推送回 GitHub。
