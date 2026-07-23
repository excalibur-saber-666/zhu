# IMU 预积分实施与验证报告

## 结论

已在既有 3 僚机 + 3 长机的“滑动窗口 + CUSUM”代码上新增独立的 IMU 预积分滑窗分支。
原有单历元 Equal-FGO、位置状态滑窗和 CUSUM 核心函数均保留；只有显式设置
`cfg.sliding_window_motion_model = 'imu_preint'` 时才会进入新分支。

本阶段证明了高频 IMU 缓存、预积分、15 状态僚机节点、预积分因子、协方差白化和三方法公平
对比流程可以运行。它不证明预积分在所有参数下精度更高，也不把本实现称为完整 fixed-lag
边缘化器。

## 实现依据与工程映射

任务说明引用的 Li 等 2025 方法要求僚机状态含位置、速度、姿态、陀螺零偏和加速度计零偏，
并以 IMU 预积分连接相邻关键帧。引用论文 PDF 不在当前工作区或下载目录，因此本报告仅依据
任务说明提供的状态与因子要求实现，不虚构论文页码或公式编号。

现有工程调用关系经检查如下：

- `IMUout.m` 输出机体系陀螺角速度，单位为度/秒，以及机体系比力，单位为 m/s²；
- `run_stage1_cusum_comparison.m` 将 IMU 误差叠加到陀螺和加速度计，再由
  `atti_cal_cq_modi.m`、`velo_cal.m`、`posi_cal.m` 进行 SINS 推算；
- `atti_cal_cq_modi.m` 接收度/秒，内部转换为 rad/s；`velo_cal.m` 使用 ENU 速度，重力方向为
  `[0;0;-9.7803698] m/s²`；
- 新分支缓存已应用当前 KF/SINS 零偏修正的输入。陀螺在缓存时从度/秒转换为 rad/s；图中的
  `b_g`、`b_a` 因而表示相对此输入的残余零偏，初始先验为零；
- CUSUM 仍在测距进入图之前计算；测距权重和报警边准入规则与原滑窗路径相同，预积分模块不接收
  故障边、故障时间、故障偏置或真值。

## 状态、坐标与因子

所有位置和速度均在局部 ENU 下表达。长机状态为 `p_L ∈ R³`。第 `k` 帧的僚机状态为：

```text
x_F,k = [p_k; v_k; theta_k; b_g,k; b_a,k] ∈ R^15
```

其中 `p` 单位 m，`v` 单位 m/s，`theta` 是 rad 的右 SO(3) 扰动，`b_g` 单位 rad/s，`b_a`
单位 m/s²。姿态矩阵 `R` 将机体系向量变换至 ENU，更新严格为：

```text
R_new = R_old * Exp(delta_theta)
```

预积分量不含重力：

```text
Δp, Δv, ΔR, Δt, Cov, sqrt_info, J_R_bg, J_v_bg, J_v_ba, J_p_bg, J_p_ba
```

预积分因子残差顺序为 `[r_p; r_v; r_R; r_bg; r_ba]`，其中重力仅在从 `i` 到 `j` 的 ENU 状态
传播项中使用。预积分协方差使用一阶离散传播并 Cholesky 白化；测距因子继续只作用于位置子状态，
并严格沿用 `sqrt(weight) / sigma_dis`，没有重复白化。

## 修改文件

新增：

- `so3_hat.m`、`so3_exp.m`、`so3_log.m`、`so3_right_jacobian.m`、`so3_right_jacobian_inverse.m`；
- `imu_preintegration_default_noise.m`、`imu_preintegrate_interval.m`、
  `imu_preintegration_residual.m`、`imu_preintegration_numeric_jacobian.m`；
- `factor_graph_sliding_window_imu_preint.m`：独立 15 状态滑窗图，不修改旧位置图；
- `run_stage1_imu_preintegration_comparison.m`：三方法比较及三轴误差图入口；
- 六个预积分/集成测试文件。

修改：

- `stage1_cusum_default_config.m`：新增预积分模式、窗口、噪声、先验、雅可比和重传播参数；
- `run_stage1_cusum_comparison.m`：缓存每个关键帧区间的高频 IMU，构造预积分窗口条目，选择独立图
  分支并记录预积分诊断；
- `使用说明_第一阶段_CUSUM.md`：新增运行说明。

## 噪声和零偏处理

噪声不凭空设置：`imu_err_random.m` 中陀螺白噪声为每个 `dt` 样本 `10 deg/h`，加速度计一阶
Markov 误差的稳态标准差为 `0.001 g`、相关时间 1800 s；陀螺 Markov 相关时间为 3600 s。配置将
每样本噪声除以 `sqrt(dt)` 映射为连续密度，并使用 `sqrt(2/T)*sigma` 作为对应零偏随机游走密度。
零偏超过配置阈值时可使用保留的原始区间样本重新预积分；本次完整运行没有依赖此机制来制造精度
结果。

## 实际测试

以下测试已实际执行并通过：

```text
test_imu_preintegration_zero_motion
test_imu_preintegration_constant_acceleration
test_imu_preintegration_constant_rotation
test_imu_preintegration_partition_consistency
test_imu_preintegration_jacobian
test_stage1_3f3l_imu_preint_smoke
test_stage1_cusum_components
test_stage1_cusum_smoke
test_factor_graph_sliding_window
test_stage1_3f3l_smoke
```

quick 集成测试使用种子 23、`[2,4]`、5–12 s、`+5 m` 故障以及 3 帧预积分窗口。每架僚机每个
关键帧区间缓存 50 个样本，`delta_t = 1.000000 s`；CUSUM 故障边权重发生变化，所有 RMSE 有限，
图优化没有未收敛帧。

完整测试实际执行的命令等价于：

```matlab
cfg = stage1_cusum_3f3l_config('full');
cfg.seeds = 23;
cfg.fault_enable = true;
cfg.fault_edge = [2,4];
cfg.fault_start = 100;
cfg.fault_end = 150;
cfg.fault_bias = 3;
cfg.imu_preint_window_length = 3;
cfg.plot_component_comparison = false;
report = run_stage1_imu_preintegration_comparison(cfg);
```

## 完整运行结果：种子 23

下表为 100–150 s 故障窗口三维 RMSE，单位 m。三种方法使用相同种子和相同随机输入序列；入口
已逐元素验证两次运行中的原始 Equal-FGO 基线一致。

| 方法 | Follower1 | Follower2 | Follower3 |
| --- | ---: | ---: | ---: |
| 原始单历元 FGO | 3.115165 | 2.745596 | 1.971499 |
| 滑窗 + CUSUM + SINS 位置增量 | 0.821371 | 1.070852 | 0.660248 |
| 滑窗 + CUSUM + IMU 预积分 | 0.840617 | 1.050144 | 0.667123 |

相对原始方法，预积分分支在此单种子工况下降低了 73.02%、61.75% 和 66.16%。与现有 SINS
位置增量分支相比，Follower2 略低，Follower1 和 Follower3 略高；因此不能声称预积分已经整体
优于原滑窗。预积分分支保持了 CUSUM 的故障处理，但精度仍需要多种子、多故障边评估。

运行诊断：SINS 比较调用总计 38.862 s；预积分比较调用总计 136.110 s。这两个数字均包含健康和
故障场景，以及同次调用中的原始基线，不能解释为单一算法每帧耗时。故障场景预积分累计计算时间为
7.134 s，平均 50.00 个 IMU 样本/区间、平均预积分时长 1.000000 s、平均 GN 迭代 6.605、未收敛
帧数 0。

`run_stage1_imu_preintegration_comparison` 在 `cfg.plot_component_comparison = true` 时会为每架僚机
生成东、北、天三轴误差图（原始/SINS/预积分三条曲线）。本次批处理数值验证未启用绘图，也没有
保存图或数据文件。

## 已知限制

- 图使用数值中心差分雅可比和稠密矩阵，完整运行明显慢于旧 SINS 位置滑窗；
- 预积分协方差为一阶近似，尚未实现解析雅可比或严格边缘化；
- 窗口长度默认 3 帧，目的是控制首版状态规模；可后续试验 5 或 10 帧；
- 当前仍使用工程既有 GPS 位置先验，而不是完整 GNSS 伪距模型；
- 本报告仅有一个完整种子结果，不能将其当作统计结论；
- 论文 PDF 未在当前可访问目录中找到，故未对论文文字/公式做逐页核验。

## 回退点

IMU 预积分改动前的完整 3F3L 基线已保存为 Git 提交 `9ef36f6` 和标签
`before-imu-preintegration-3f3l`。该标签可用于回退。
