# 3F3L 协同导航四方法对比实验

这是一个 MATLAB 仿真项目，用于比较三架僚机（Follower 1--3）和三架长机（Leader 1--3）组成的协同导航网络。在同一组 IMU、GPS、测距噪声和测距故障下，项目比较 EKF、FGO、CUSUM-EKF 与 CUSUM-FGO 四种方法。

所有 MATLAB 源文件、核心测试和初始位置数据均平铺在项目根目录；不要重新按 `src/`、`tests/` 等目录拆分。论文草稿、下载的文献 PDF 和本地 Monte Carlo 结果均被忽略，不会提交到 GitHub。

## 论文使用的实验

当前论文图、表和 50 次 Monte Carlo 统计**只能**使用原始配置 `li_style_dense`。它包含 600 s 仿真、0.02 s IMU 更新、1 s 图优化和 13 段 3/4/6 m 的僚机--长机测距偏差，其中既有单边故障，也有不同僚机上的同时故障。

运行一组代表性试验并显示 MATLAB 图：

```matlab
setup_project
report = run_li_style_comparison(23, true, 'li_style_dense');
```

运行论文的 50 次 Monte Carlo 统计与出图：

```matlab
setup_project
summary = run_li_style_paper_experiment(1:50, 23, [], 'li_style_dense');
```

第二个命令会在根目录生成本地结果目录 `li_style_mc50_results/`，其中包含 CSV、XLSX 和 MATLAB 图片；该目录已被 `.gitignore` 排除。

## 四个对照组

| 方法 | 图结构/估计器 | CUSUM 处理 | 用途 |
| --- | --- | --- | --- |
| `EKF` | 固定权重的协同 18 状态 EKF | 无 | 传统滤波基线 |
| `FGO` | 单历元、等权因子图 | 无 | 传统因子图基线 |
| `CUSUM-EKF` | 协同 EKF | 在线双侧 CUSUM、软降权、确认后选择性隔离 | EKF 消融对照 |
| `CUSUM-FGO` | 10 帧滑动窗口因子图，含 SINS 相对位移因子 | 在线双侧 CUSUM、软降权、确认后选择性隔离 | 论文主方法 |

四个方法由同一个缓存的随机输入驱动，因而不会因 IMU、GPS、测距噪声或故障调度不同而产生不公平比较。在线检测函数没有真值、故障边、故障时段或故障幅值的输入；这些信息只用于离线注入和统计。

更完整的算法边界、故障表、公式和文件映射见 [EXPERIMENT_CONTEXT.md](EXPERIMENT_CONTEXT.md)。该文件是交给 GPT 或后续开发者理解本项目时应优先阅读的说明。

## 单独运行一个消融组

```matlab
setup_project
cfg = stage1_cusum_3f3l_config('li_style_dense');
cfg.seeds = 23;
cfg.plot_position_results = true;

report = main_stage1('ekf', cfg);
report = main_stage1('fgo', cfg);
report = main_stage1('cusum_ekf', cfg);
report = main_stage1('cusum_fgo', cfg);
```

快速冒烟检查可改用 `stage1_cusum_3f3l_config('quick')`；它缩短仿真时间，不可作为论文结果。

## 保留但未用于论文的改进配置

以下配置是为研究调参而保留的代码路径，**未用于当前论文的任何图、表或 50 次统计，不能与 `li_style_dense` 的结果混用**：

- `li_style_dense_tuned`：对同一僚机下所有可疑长机边进行软降权，并给 CUSUM-EKF 启用低信息预测测距替代项。
- `li_style_dense_fgo_recovered`：在 `tuned` 基础上，当在线统计确认持续反向突变时重置 FGO 检测器的测距预测器，以避免陈旧告警长期占用健康几何。

例如，仅作研究复核时：

```matlab
report = run_li_style_comparison(23, true, 'li_style_dense_tuned');
```

## 可选 IMU 预积分

IMU 预积分作为研究变体保留，默认关闭，也不属于当前论文四方法结果。开启后，它只替换 CUSUM-FGO 的 SINS 相对位移因子，不改变另外三个对照组：

```matlab
cfg.imu_preintegration_enable = true;
report = main_stage1('cusum_fgo', cfg);
```

## 核心入口与测试

- `setup_project.m`：将当前根目录加入 MATLAB 路径。
- `main_stage1.m`：四方法或单方法的统一入口。
- `stage1_cusum_3f3l_config.m`：3F3L 场景、论文故障调度和备用改进配置。
- `run_stage1_cusum_comparison.m`：生成共享随机输入并执行各估计器。
- `run_li_style_comparison.m`：单一代表性种子入口。
- `run_li_style_paper_experiment.m`：Monte Carlo、统计表和 MATLAB 图入口。

运行保留的核心检查：

```matlab
results = run_core_tests;
```

测试覆盖 CUSUM 的在线性与告警逻辑、四方法共享输入及单方法入口，以及默认关闭的 IMU 预积分变体。
