# 实验与算法说明（供 GPT 和后续维护者阅读）

## 1. 这份代码在做什么

项目在 ENU 局部坐标系中仿真 6 架无人机：节点 1--3 为僚机（Follower），节点 4--6 为长机（Leader）。每架僚机都具有 SINS/IMU 和 GPS 信息，并与通信范围内的其他节点进行测距。研究目标是在测距出现持续偏差时，对比传统协同导航与基于 CUSUM 的鲁棒协同导航。

测距观测的代码语义为

\[
z_{ij,k}=\lVert \mathbf p_{i,k}-\mathbf p_{j,k}\rVert+v_{ij,k}+b_{ij,k},
\]

其中 \(v_{ij,k}\sim\mathcal N(0,\sigma_{dis}^2)\)，且当前 Stage-1 验证锁定 \(\sigma_{dis}=0.2\) m。\(b_{ij,k}\) 仅由离线故障注入器 `local_inject_fault` 写入；任何在线 CUSUM 或估计函数均不接收它。

三维定位误差和 RMSE 的定义为

\[
e_{p,k}=\sqrt{(\hat E_k-E_k)^2+(\hat N_k-N_k)^2+(\hat U_k-U_k)^2},\qquad
\operatorname{RMSE}=\sqrt{\frac1N\sum_{k=1}^{N}e_{p,k}^2}.
\]

## 2. 论文固定配置与结果边界

论文采用：

```matlab
cfg = stage1_cusum_3f3l_config('li_style_dense');
```

这是唯一可以用于当前论文图片、表格和 50 次 Monte Carlo 统计的 profile。它使用 600 s 仿真、`dt = 0.02` s、`graph_interval = 1` s、3 架僚机和 3 架长机；定位图的代表性种子通常为 23，统计为 `1:50`。故障模式为 `segmented_explicit_edges`，每行格式为：

```text
[start_s, end_s, additive_bias_m, follower, leader_global]
```

| 时段 (s) | 偏差 (m) | 故障边 |
| --- | ---: | --- |
| 80--92 | +3 | F1--L1 |
| 120--135 | +4 | F2--L2 |
| 170--185 | +3 | F3--L3 |
| 245--262 | +4 | F1--L2 |
| 245--262 | +6 | F2--L1 |
| 285--302 | +6 | F3--L1 |
| 310--326 | +4 | F2--L3 |
| 365--380 | +3 | F1--L3 |
| 365--380 | +4 | F3--L2 |
| 430--447 | +6 | F2--L2 |
| 485--500 | +3 | F1--L1 |
| 525--545 | +4 | F3--L3 |
| 535--552 | +6 | F2--L1 |

同一时段有多条故障边时，它们属于不同僚机；每一僚机在任一时刻最多有一条故障长机边，从而保留其余长机测距以执行“选择性隔离”。

`li_style_dense_tuned` 和 `li_style_dense_fgo_recovered` 是保留的改进实现，目的是后续研究复核，未用于论文结果；阅读或写论文时不得把它们的数值、图片或结论与原始 `li_style_dense` 混合。

## 3. 四个公平对照组

`main_stage1('four_method', cfg)` 固定运行以下四个分支。`run_stage1_cusum_comparison.m` 先为每个种子生成一份共享缓存，再将相同缓存和相同故障调度提供给全部分支。

| 名称 | `estimator_mode` | `graph_mode` | CUSUM | 解释 |
| --- | --- | --- | --- | --- |
| EKF | `ekf` | `single_epoch` | 关闭 | 固定测距权重的局部 18 状态协同 EKF。 |
| FGO | `fgo` | `single_epoch` | 关闭 | 单历元、等权的传统因子图。 |
| CUSUM-EKF | `ekf` | `single_epoch` | 开启 | 测距预测器 CUSUM，软降权和确认后隔离。 |
| CUSUM-FGO | `fgo` | `sliding_window` | 开启 | 10 帧位置滑动窗口、SINS 相对位移因子、CUSUM 软降权和确认后隔离；是论文主方法。 |

所有 FGO 分支在每个图历元使用长机和僚机 GPS 位置先验与测距因子；CUSUM-FGO 额外连接相邻帧的 SINS 相对位移因子。图优化结果再通过 `kalm_factor_measure_update.m` 反馈到各僚机的导航解。对照边界因此是“单历元等权 FGO”对“滑动窗口 CUSUM-FGO”，而不是两个仅在 CUSUM 开关上不同的同构图。

## 4. 论文 profile 的在线 CUSUM

`li_style_dense` 明确设置 `fgo_cusum_detector_mode = 'range_predictor'`。因此 CUSUM-EKF 与 CUSUM-FGO 均调用 `compute_ekf_cusum_range_weights.m` 的独立逐边 alpha-beta 测距预测器；二者**共享检测统计，但在不同的 EKF/因子图估计器中使用权重**。它的预测不依赖 EKF 或图优化已融合的测距结果，可避免估计器反馈抵消检测残差。

对每条边，预测、归一化创新和双侧折扣 CUSUM 为

\[
\hat d^-_k=\hat d_{k-1}+\dot{\hat d}_{k-1}\Delta t,\qquad
r_k=z_k-\hat d^-_k,
\]

\[
z_k^{\mathrm{CUSUM}}=\frac{r_k-\mu_k}{\sqrt{\max(s_k^2,\sigma_{\min}^2)}},\qquad
C_k^+=\max\{0,\lambda C_{k-1}^++z_k^{\mathrm{CUSUM}}-\kappa\},
\]

\[
C_k^-=\max\{0,\lambda C_{k-1}^--z_k^{\mathrm{CUSUM}}-\kappa\},\qquad
C_k=\max\{C_k^+,C_k^-\}.
\]

健康标定阶段为每条边独立学习 \(\mu_k\) 与 \(s_k^2\)。报警期间预测器冻结其距离基准，并仅在差分速率通过门限时缓慢更新速率，避免把持续加性偏差吸收为正常变化。若报警后出现持续且足够强的反向在线创新，`li_style_dense_fgo_recovered` 可选择重置预测器；原始论文 profile 中该功能关闭。

软权重为

\[
w_k=\operatorname{clip}_{[w_{\min},1]}\!\left[
\frac{1}{1+g\left(\max(0,C_k-d)+\max(0,|z_k^{\mathrm{CUSUM}}|-\tau)^2\right)}
\right].
\]

其中 \(d\) 是权重死区，\(\tau\) 是创新门限。原始 profile 的默认 `per_follower_max` 策略只对同一僚机下 CUSUM 最大的可疑长机边施加软降权。CUSUM 超过开阈值并满足连续确认历元数后，`per_follower_max` 选择性隔离该僚机下 CUSUM 最大的一条报警长机边；其余健康边继续进入估计器。告警在近似正常创新连续满足释放条件后解除。

`compute_signed_cusum_edge_weights.m` 是另一种基于位置先验残差的通用 CUSUM 实现；仅当 `fgo_cusum_detector_mode = 'prior_innovation'` 时使用，因而不属于 `li_style_dense` 论文 profile。

## 5. 权重如何进入 EKF 与 FGO

### CUSUM-EKF

`cooperative_range_kf_update.m` 对每架僚机将当前图历元的相邻测距一次性批处理。测距权重通过有效测量方差进入：

\[
R_{ij}^{\mathrm{eff}}=\frac{\sigma_{dis}^2}{w_{ij}}+
\mathbf u_{ij}^{\mathsf T}\mathbf P_j\mathbf u_{ij},
\qquad
\mathbf u_{ij}=\frac{\mathbf p_i-\mathbf p_j}{\lVert\mathbf p_i-\mathbf p_j\rVert}.
\]

第二项传播邻居位置协方差；各节点均在本历元 GPS 更新后冻结，因此批处理结果不依赖测距边的遍历顺序。确认报警后，原始坏测距不进入更新。备用 `tuned` 配置可用预测距离替代该边，并赋予低信息权重；论文 profile 禁用该替代项。

### CUSUM-FGO

`factor_graph_sliding_window.m` 为每条保留的测距因子使用一次且仅一次的白化系数：

\[
\tilde r_{ij}=\frac{\sqrt{w_{ij}}}{\sigma_{dis}}
\left(\lVert\mathbf p_i-\mathbf p_j\rVert-z_{ij}\right).
\]

窗口长度为 10 帧。新的测距到达时已确定它的权重与是否准入；此决定会随该测距因子保留在窗口中，而不会用未来信息重新计算。确认报警的边不写入新帧，原有因子会随窗口滑出。请勿在任何改动中对同一测距权重重复乘以 \(\sqrt w\)。

## 6. 代码入口与修改规则

| 文件 | 责任 |
| --- | --- |
| `setup_project.m` | 加入根目录 MATLAB 路径。 |
| `main_stage1.m` | 选择四方法或单方法。 |
| `stage1_cusum_default_config.m` | 公共时间、噪声、图窗、CUSUM、EKF 和 IMU 预积分参数。 |
| `stage1_cusum_3f3l_config.m` | 3F3L 节点映射、论文 13 段故障及备用改进 profile。 |
| `run_stage1_cusum_comparison.m` | 缓存随机输入、注入离线故障、路由四种算法、输出诊断与指标。 |
| `compute_ekf_cusum_range_weights.m` | 论文 profile 实际使用的 alpha-beta + 双侧 CUSUM + 权重/告警。 |
| `cooperative_range_ekf_update.m` | EKF 的加权批量测距更新入口。 |
| `factor_graph_centralization.m` | FGO 的单历元图。 |
| `factor_graph_sliding_window.m` | 论文 CUSUM-FGO 的位置滑动窗口图。 |
| `factor_graph_sliding_window_imu_preint.m` | 保留的 15 状态 IMU 预积分研究变体，默认关闭且不用于论文。 |
| `run_li_style_paper_experiment.m` | 50 次 Monte Carlo、数据表和 MATLAB 图片。 |

若后续改算法：保持四方法复用同一随机输入；不得让在线检测读取真值或故障元数据；不得把 `li_style_dense_tuned` 或 `li_style_dense_fgo_recovered` 的结果写进当前论文；也不要重新生成或提交本地结果目录、论文文档和下载文献。

## 7. 推荐命令

```matlab
% 单个代表性种子和图片
setup_project
report = run_li_style_comparison(23, true, 'li_style_dense');

% 正式 50 次 Monte Carlo
summary = run_li_style_paper_experiment(1:50, 23, [], 'li_style_dense');

% 只跑主方法 CUSUM-FGO
cfg = stage1_cusum_3f3l_config('li_style_dense');
cfg.seeds = 23;
cfg.plot_position_results = true;
report = main_stage1('cusum_fgo', cfg);

% 核心回归检查
results = run_core_tests;
```
