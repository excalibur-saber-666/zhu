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
