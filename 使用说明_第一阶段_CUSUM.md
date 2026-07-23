# 第一阶段 CUSUM 协同导航使用说明

## 1. 当前固定实验配置

第一阶段保持原始工程的 5 架无人机场景：3 架长机（高精度节点）和 2 架僚机（待定位节点）。初始位置继续从以下原始文件读取，飞行轨迹、速度和转弯时序均未修改：

```text
posi_e_all.dat
posi_n_all.dat
posi_u_all.dat
```

节点编号含义如下：

| 全局编号 | 节点 |
| ---: | --- |
| 1--2 | Follower1--Follower2（僚机） |
| 3--5 | Leader1--Leader3（长机） |

默认故障边 `cfg.fault_edge = [2, 3]` 表示 Follower2--Leader1。

## 2. 运行入口

先在 MATLAB 中将当前目录切换到本工程根目录，再运行：

```matlab
report = run_stage1_cusum_comparison();
```

程序在相同的随机缓存输入下比较：

- `Equal-FGO`：所有有效测距边等权；
- `CUSUM-FGO`：根据在线创新统计对异常测距边连续软降权。

不绘图、不保存 MAT 文件、不导出表格；结果显示在 MATLAB 命令行，并保存在变量 `report` 中。

## 3. 常用运行方式

### 健康场景

```matlab
cfg = stage1_cusum_default_config('full');
cfg.seeds = 23;
cfg.fault_enable = false;
report = run_stage1_cusum_comparison(cfg);
```

### 3 m、50 s 测距故障

```matlab
cfg = stage1_cusum_default_config('full');
cfg.seeds = 23;
cfg.fault_enable = true;
cfg.fault_edge = [2, 3];
cfg.fault_start = 100;
cfg.fault_end = 150;
cfg.fault_bias = 3;
report = run_stage1_cusum_comparison(cfg);
```

### 快速检查

```matlab
cfg = stage1_cusum_default_config('quick');
cfg.seeds = 23;
report = run_stage1_cusum_comparison(cfg);
```

`full` 为完整 600 s 仿真，适合正式结果；`quick` 为 20 s 烟雾测试，只用于检查程序是否能运行。

## 4. 如何修改故障

```matlab
cfg.fault_enable = true;
cfg.fault_edge = [2, 3];
cfg.fault_start = 100;
cfg.fault_end = 150;
cfg.fault_bias = 3;
```

`fault_end` 必须大于 `fault_start`。故障只注入测距观测；在线 CUSUM 不读取真值、故障边、故障时间、故障幅值或 `fault_enable`。

## 5. 如何理解输出

每架僚机单独输出以下指标：

- `Full RMSE(m)`：全时段三维 RMSE；
- `Fault-window RMSE(m)`：按 `cfg.fault_start` 到 `cfg.fault_end` 统计；
- `Full vs Equal`：CUSUM 相对 Equal 的全时段 RMSE 变化；
- `Window vs Equal`：CUSUM 相对 Equal 的故障窗口 RMSE 变化。

负百分比代表 CUSUM-FGO 优于 Equal-FGO。多随机种子时，主表输出平均值，随后输出标准差表。

## 6. 健康保护的默认设置

为避免健康场景中由 SINS/KF 共同预测失配造成的误降权，默认 CUSUM 会比较同一僚机相连的其他已标定测距边。这是在线边间一致性统计，不读取故障标签或真值。当前默认参数为：

```matlab
cfg.cusum_consensus_enable = true;
cfg.cusum_lambda = 0.80;
cfg.cusum_kappa = 0.35;
cfg.cusum_weight_deadzone = 2.00;
```

不要在不知道影响的情况下单独降低死区或提高遗忘因子，否则健康边可能重新被频繁降权。

## 7. 已验证结果与限制

当前默认参数在独立种子 `6:20` 的健康场景中，CUSUM 相对 Equal 的全时段 RMSE 变化为 Follower1 `+0.39%`、Follower2 `+0.75%`；与此前健康段明显变差的设置相比，误降权已大幅减少。

在训练种子 `1:5` 上，默认故障边 `[2,3]` 的故障窗口 RMSE 相对 Equal 变化如下：

| 故障 | Follower1 | Follower2 |
| --- | ---: | ---: |
| `+3 m, 50 s` | `-5.28%` | `-14.46%` |
| `+5 m, 50 s` | `-12.58%` | `-24.38%` |
| `+8 m, 10 s` | `-59.20%` | `-67.84%` |
| `-3 m, 50 s` | `-2.99%` | `-10.51%` |
| `-5 m, 50 s` | `-5.91%` | `-17.25%` |

另一条边 `[1,3]` 的 `+5 m` 试验基本持平（`-0.09% / +0.06%`）。因此当前版本不能宣称对每一条边、每一种几何关系都已稳定改善；运行新边或新轨迹前应重新验证。

## 8. 测试

```matlab
test_stage1_cusum_components
test_stage1_cusum_smoke
```

看到以下输出表示通过：

```text
test_stage1_cusum_components: PASS
test_stage1_cusum_smoke: PASS
```

## 9. 说明

本阶段固定保留原始 5 机、3 长机、2 僚机的实验场景；如需研究其他无人机数量或近距编队，应另建独立实验配置，避免与当前 CUSUM 基线结果混在一起。

## 10. 原始混合导航脚本的滑动窗口

`hybrid_cooperative_navigation.m` 已增加集中式位置滑动窗口；其运行入口仍然是：

```matlab
hybrid_cooperative_navigation
```

默认设置位于脚本的“滑动窗口因子图参数”段：

```matlab
window_length = 10;
window_motion_std = [2;2;4];
```

窗口每秒加入一个关键帧，最多保留最近 10 帧。每帧包含长机/僚机位置先验和测距因子；相邻帧的僚机状态由 SINS 相对位移因子相连。因此历史测距会经由运动约束影响当前帧，而不是错误地直接施加到当前状态。

新增类 `factor_graph_sliding_window.m` 仅服务于原始 `hybrid_cooperative_navigation.m`；原来的 `factor_graph_centralization.m` 和 Stage-1 CUSUM 比较入口均未替换。飞行轨迹、KF/SINS 方程和传感器设置没有改动。

可先运行新增单元测试：

```matlab
test_factor_graph_sliding_window
```

完整原始脚本仍会按原有逻辑绘图并写入 `*_hybrid*.dat` 仿真输出文件。

## 11. 原始 FGO 与“滑动窗口 + CUSUM”对比图

`hybrid_cooperative_navigation.m` 直接运行时只使用滑动窗口，并**不**启用 CUSUM。
若要公平比较“原始单时刻等权 FGO”与“滑动窗口 + CUSUM-FGO”，请使用新增入口：

```matlab
cfg = stage1_cusum_default_config('full');
cfg.seeds = 23;

cfg.fault_enable = true;
cfg.fault_edge = [2, 3];
cfg.fault_start = 100;
cfg.fault_end = 150;
cfg.fault_bias = 3;

cfg.sliding_window_length = 10;
cfg.sliding_window_motion_std = [2; 2; 4];
cfg.sliding_window_exclude_alarmed_edges = true;
cfg.sliding_window_cusum_consensus_enable = false;

report = run_stage1_sliding_window_cusum_comparison(cfg);
```

运行后会为每架僚机分别生成一张三子图：东向、北向、天向位置误差。

- 蓝线：原始单时刻、等权 FGO；
- 红线：滑动窗口 + 在线 CUSUM 权重 FGO；
- 黑色虚线：故障起止时刻（仅在 `fault_enable = true` 时出现）。

两种方法在同一随机种子下使用完全相同的 IMU、GPS 和测距随机噪声；唯一的设计差异是，红线方法保留最近 `sliding_window_length` 个关键帧，并对每条新到达的测距边使用在线 CUSUM 权重。窗口内的历史边保留其到达当时确定的权重，不使用未来信息回算。

当某条边的在线 CUSUM 进入报警状态时，`sliding_window_exclude_alarmed_edges = true` 会停止把该边写入新的窗口帧；已经存在的旧因子不会被回溯删除，而是随窗口正常移出。因此该规则不使用 `fault_edge`、故障时间或真值，并能避免持续故障在窗口中反复传播。若要仅研究软降权，可显式设为 `false`。

该五机几何中，每架僚机只有两条可替代的长机边。对单一故障直接取两条边的中位数会把故障值混入参考量，可能使健康边被误降权。因此组合入口默认 `sliding_window_cusum_consensus_enable = false`，使用每条边已标定的在线创新 CUSUM；原有 `run_stage1_cusum_comparison` 的单时刻实验仍保留原本的共识设置。

不设置故障时，可仅比较健康段：

```matlab
cfg = stage1_cusum_default_config('full');
cfg.seeds = 23;
cfg.fault_enable = false;
report = run_stage1_sliding_window_cusum_comparison(cfg);
```

这个入口会在命令行输出每架僚机的全时段 RMSE、故障窗口 RMSE、相对原始 FGO 的变化百分比；多种子时还会输出标准差、最差种子、CUSUM 权重与 GN 收敛诊断。同时画三方向对比图，不写入 `*_hybrid*.dat`。如不需要保存返回变量，仍建议在调用末尾添加分号：

```matlab
run_stage1_sliding_window_cusum_comparison(cfg);
```

快速冒烟检查（不画图）可运行：

```matlab
test_stage1_sliding_window_cusum_smoke
```

## 12. 六机冗余长机 + 选择性报警边剔除（独立实验）

原始五机基线不会被替换。如果要验证“同一僚机有多条报警长机边时，
只暂停最可疑的一条、其余长机边继续保留”的方案，请使用独立的六机配置：

```matlab
cfg = stage1_cusum_redundant_config('full');
cfg.seeds = 23;

cfg.fault_enable = true;
cfg.fault_edge = [2, 3];
cfg.fault_start = 100;
cfg.fault_end = 150;
cfg.fault_bias = 3;

report = run_stage1_sliding_window_cusum_comparison(cfg);
```

这会同时输出命令行 RMSE、CUSUM 诊断表，并为两架僚机绘制东、北、天三方向误差图。若只想先确认程序能运行，可改用：

```matlab
cfg = stage1_cusum_redundant_config('quick');
cfg.seeds = 23;
run_stage1_sliding_window_cusum_comparison(cfg);
```

六机配置的节点编号为：`1--2` 是 Follower1--Follower2，`3--6` 是 Leader1--Leader4。
Leader4 的初始局部 ENU 偏置为 `[600; 250; 300] m`，之后使用与原编队相同的既定机动；原有五架飞机的位置数据、飞行轨迹和噪声设置均不修改。

该配置默认启用以下两项专门用于冗余长机的规则：

```matlab
cfg.cusum_consensus_enable = true;
cfg.cusum_consensus_leader_only = true;
cfg.cusum_consensus_min_neighbors = 3;
cfg.sliding_window_alarm_exclusion_mode = 'per_follower_max';
```

`per_follower_max` 的含义是：在每一个关键帧，对每架僚机单独检查其报警的“僚机--长机”测距边；只将 CUSUM 值最大的那一条停止写入新的滑动窗口帧。其他长机边，以及僚机--僚机边，都会继续保留。已经进入窗口的历史因子不会被回溯删除，只会随窗口长度正常移出。

如果你要恢复“所有报警边均不进入新窗口”的旧策略，可显式设置：

```matlab
cfg.sliding_window_alarm_exclusion_mode = 'all';
```

六机选择性策略的短时回归检查为：

```matlab
test_stage1_redundant_sliding_window_cusum_smoke
```

该测试会逐关键帧验证：每架僚机最多排除一条长机边，而且被排除的边就是该僚机当前 CUSUM 值最大的报警边。

## 13. 审计后的消融与数值诊断

当前滑窗是固定长度的移动窗口批量优化；最老帧直接移出，并没有 Schur 补边缘化。
运动约束是经验 SINS 位置增量约束，不是严格的 IMU 预积分因子。可用以下开关做消融：

```matlab
cfg.sliding_window_motion_enable = false; % 关闭跨帧 SINS 增量约束
cfg.sliding_window_exclude_alarmed_edges = false; % 仅 CUSUM 软降权，不做硬隔离
cfg.graph_condition_diagnostics = true; % 记录每帧秩、奇异值、条件数和求解时间
```

等权滑窗与单历元的严格回归模式为：

```matlab
cfg = stage1_cusum_redundant_config('quick');
cfg.seeds = 23;
cfg.fault_enable = false;
cfg.verbose = false;
cfg.comparison_mode = 'original_vs_sliding_equal';
cfg.sliding_window_length = 1;
cfg.sliding_window_motion_enable = false;
cfg.sliding_window_exclude_alarmed_edges = false;
report = run_stage1_cusum_comparison(cfg);
```

建议直接运行以下回归：

```matlab
test_stage1_window_length_one_regression
test_stage1_sliding_without_motion_regression
test_stage1_redundant_geometry
```

完整的审计结论、三种子消融结果和当前限制见 `阶段一_6机滑窗CUSUM_审计报告.md`。

## 14. 如何切换对比模式

需要自由切换时，请使用通用入口 `run_stage1_cusum_comparison(cfg)`，并只修改
`cfg.comparison_mode`。三个模式如下：

| `cfg.comparison_mode` | 左侧方法 | 右侧方法 | 用途 |
| --- | --- | --- | --- |
| `'equal_vs_cusum'` | 单历元 Equal-FGO | 单历元 CUSUM-FGO | 单独验证 CUSUM 权重 |
| `'original_vs_sliding_cusum'` | 原始单历元 Equal-FGO | 当前滑窗 + CUSUM + 在线隔离 | 原始方法与现在改进方法的主对比 |
| `'original_vs_sliding_equal'` | 原始单历元 Equal-FGO | 滑窗等权 FGO | 审计/消融，不使用 CUSUM |

例如，要比较故障边 `(1,6)` 上的“最原始方法”与“现在改进方法”，运行：

```matlab
cfg = stage1_cusum_redundant_config('full');
cfg.seeds = [21, 22, 23];
cfg.fault_enable = true;
cfg.fault_edge = [1, 6];
cfg.fault_start = 100;
cfg.fault_end = 150;
cfg.fault_bias = 5;

cfg.comparison_mode = 'original_vs_sliding_cusum';
cfg.plot_component_comparison = true;
report = run_stage1_cusum_comparison(cfg);
```

若只比较两个都是单历元的方法，将最后两行改为：

```matlab
cfg.comparison_mode = 'equal_vs_cusum';
cfg.plot_component_comparison = true;
report = run_stage1_cusum_comparison(cfg);
```

`run_stage1_sliding_window_cusum_comparison(cfg)` 是一个便捷入口，它固定运行
`'original_vs_sliding_cusum'` 并自动绘图；要切换模式时不要使用该便捷入口。

## 15. 三架僚机、三架长机（3F3L）实验

原来的六机冗余配置是“两架僚机、四架长机”。要测试“三架僚机、三架长机”，请使用
`stage1_cusum_3f3l_config`，不要直接在原配置中只修改 `uav_num` 或 `high_num`。
新配置中节点 `1--3` 为僚机、节点 `4--6` 为长机；为保持原来“僚机2--长机1”故障的
含义，默认故障边已改为 `[2, 4]`，而 `[2, 3]` 在此配置中是僚机间测距边。

下面的代码比较原始单历元 FGO 与滑动窗口 + CUSUM + 在线隔离：

```matlab
cfg = stage1_cusum_3f3l_config('full');
cfg.seeds = [21, 22, 23];

cfg.fault_enable = true;
cfg.fault_edge = [2, 4];       % Follower2--Leader1
cfg.fault_start = 100;
cfg.fault_end = 150;
cfg.fault_bias = 3;

cfg.comparison_mode = 'original_vs_sliding_cusum';
cfg.plot_component_comparison = true;
report = run_stage1_cusum_comparison(cfg);
```

该配置使用数据中的三组长机初始位置，使每架僚机到三架长机的九条测距边均在 500 m
通信范围内，且初始三维几何满秩。三架长机的冗余度低于原来的四架长机，因此结果不应
直接预期为更好；需要以输出的三架僚机 RMSE 分别判断。运行前可执行
`test_stage1_3f3l_smoke` 进行快速检查。

在上述配置、`seeds = [21, 22, 23]` 下，故障窗口三维 RMSE 的平均值分别为：

| 僚机 | 原始单历元 FGO (m) | 滑动窗口 + CUSUM (m) | 相对变化 |
| --- | ---: | ---: | ---: |
| Follower1 | 3.335822 | 1.240454 | -62.81% |
| Follower2 | 3.275175 | 1.111090 | -66.08% |
| Follower3 | 1.913215 | 0.749435 | -60.83% |

这组数字仅对应 `Follower2--Leader1` 的 `+3 m`、100--150 s 单边故障；更换故障边、
偏置或随机种子后，应以重新运行的汇总表为准。
