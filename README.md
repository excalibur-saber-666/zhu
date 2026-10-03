# 3F3L 协同导航四方法对比实验

这是一个 MATLAB 仿真项目，用于比较三架僚机（Follower 1--3）和三架长机（Leader 1--3）组成的协同导航网络。在同一组 IMU、GPS、测距噪声和测距故障下，项目比较 EKF、FGO、CUSUM-EKF 与 CUSUM-FGO 四种方法。

工作区按用途分为四个文件夹：

- `code/`：50 个现有 MATLAB 源文件与测试；初始位置数据位于 `code/data/`，使用说明位于 `code/说明/`。旧文档构建脚本归档在 `code/历史工具/`，不会自动加入 MATLAB 路径。
- `论文/`：英文稿、中文草稿、可编辑流程图、审稿意见与修订记录；旧稿保存在 `论文/历史版本/`。
- `结果/`：现有实验输出、历史中间文件以及本次整理的迁移清单和校验记录。后续默认结果也写入此目录。
- `资料/`：参考论文、参考文献记录及 EIC 排版模板。

根目录保留项目说明和 Git/工具配置。论文正文、下载资料、生成结果和历史构建工具只在本地保留，不提交到 GitHub。已有受版本控制的修订说明保留其跟踪状态。

下面的 MATLAB 命令从工作区根目录执行。也可以进入 `code/` 后直接调用 `setup_project`；数据路径按文件位置解析，与当前工作目录无关。

## 论文使用的实验

当前论文图、表和 50 次 Monte Carlo 统计**只能**使用配置 `li_style_dense_random_nlos`。它保留原有 13 个异常时段和僚机--长机链路；每段开始时从截断双分量高斯混合模型

\[
B_s\sim0.6\mathcal N(3,0.6^2)+0.4\mathcal N(6,1.0^2),\qquad 1\le B_s\le8\ \mathrm m
\]

独立抽取一个正偏差并在段内保持不变。异常时段外不注入 NLOS 偏置，仅保留标准差 0.2 m 的正常测距噪声。同一 Monte Carlo 种子下，四种方法共享完全相同的 IMU、GPS、测距噪声和 NLOS realization。

运行一组代表性试验并显示 MATLAB 图：

```matlab
addpath(fullfile(pwd, 'code'));
setup_project
report = run_li_style_comparison(23, true, 'li_style_dense_random_nlos');
```

运行论文的 50 次 Monte Carlo 统计与出图：

```matlab
addpath(fullfile(pwd, 'code'));
setup_project
summary = run_li_style_paper_experiment(1:50, 23, [], 'li_style_dense_random_nlos');
```

第二个命令会在 `结果` 文件夹中生成本地结果目录 `li_style_mc50_random_nlos_results/`，其中包含 CSV、XLSX、MATLAB 图片和逐段 NLOS 幅值源数据；该目录已被 `.gitignore` 排除。

## 四个对照组

| 方法 | 图结构/估计器 | CUSUM 处理 | 用途 |
| --- | --- | --- | --- |
| `EKF` | 固定权重的协同 18 状态 EKF | 无 | 传统滤波基线 |
| `FGO` | 单历元、等权因子图 | 无 | 传统因子图基线 |
| `CUSUM-EKF` | 协同 EKF | 在线双侧 CUSUM、软降权、确认后选择性隔离 | EKF 消融对照 |
| `CUSUM-FGO` | 10 帧滑动窗口因子图，含 SINS 相对位移因子 | 在线双侧 CUSUM、软降权、确认后选择性隔离 | 论文主方法 |

四个方法由同一个缓存的随机输入驱动，因而不会因 IMU、GPS、测距噪声或故障调度不同而产生不公平比较。在线检测函数没有真值、故障边、故障时段或故障幅值的输入；这些信息只用于离线注入和统计。

更完整的算法边界、故障表、公式和文件映射见 [EXPERIMENT_CONTEXT.md](code/EXPERIMENT_CONTEXT.md)。该文件是交给 GPT 或后续开发者理解本项目时应优先阅读的说明。

## 单独运行一个消融组

```matlab
addpath(fullfile(pwd, 'code'));
setup_project
cfg = stage1_cusum_3f3l_config('li_style_dense_random_nlos');
cfg.seeds = 23;
cfg.plot_position_results = true;

report = main_stage1('ekf', cfg);
report = main_stage1('fgo', cfg);
report = main_stage1('cusum_ekf', cfg);
report = main_stage1('cusum_fgo', cfg);
```

快速冒烟检查可改用 `stage1_cusum_3f3l_config('quick')`；它缩短仿真时间，不可作为论文结果。

## 保留但未用于当前论文的配置

以下配置用于历史复现或研究调参，**未用于当前论文的任何图、表或 50 次统计，不能与 `li_style_dense_random_nlos` 的结果混用**：

- `li_style_dense`：原 13 段固定 3/4/6 m 偏差，仅用于复现旧结果。
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

- `code/setup_project.m`：将有效代码目录加入 MATLAB 路径，并返回工作区根目录。
- `code/main_stage1.m`：四方法或单方法的统一入口。
- `code/stage1_cusum_3f3l_config.m`：3F3L 场景、论文故障调度和备用改进配置。
- `code/run_stage1_cusum_comparison.m`：生成共享随机输入并执行各估计器。
- `code/run_li_style_comparison.m`：单一代表性种子入口。
- `code/run_li_style_paper_experiment.m`：Monte Carlo、统计表和 MATLAB 图入口。

运行保留的核心检查：

```matlab
addpath(fullfile(pwd, 'code'));
setup_project
results = run_core_tests;
```

测试覆盖 CUSUM 的在线性与告警逻辑、四方法共享输入及单方法入口，以及默认关闭的 IMU 预积分变体。

## 2026-10-03 论文返修实验

E3→E2→E1→E4→E5 已按冻结方案完成，完整材料见 [实验结果总索引](实验结果总索引.md)。主参数 h=5、κ=0.5 保持，正式 Word 尚未修改。实验执行时按用户要求保留在本地；本次按用户后续明确要求，将实验修改提交并上传 GitHub。

新增正式入口为 `run_revision_experiments`、`run_remaining_revision_experiments` 与对应配置工厂；两阶段结果分别在 `结果/2026-10-03_E3_E2_revision/` 和 `结果/2026-10-03_E1_E4_E5_revision/`，复算使用各自执行快照。新增计时与机制消融默认关闭，保留当前基准。实现、状态和验证说明见 [返修实验实现与验证](code/说明/2026-10-03_返修实验实现与验证.md)。

GitHub 包含实验实现、配置、测试、统计分析、绘图、报告生成脚本及修改方案和实现说明。两阶段报告、汇总 CSV、主要图表和已有验收记录另以 [Release 审阅包](https://github.com/excalibur-saber-666/zhu/releases/tag/revision-experiments-2026-10-03) 提供，下载与核查步骤见 [实验结果下载说明](实验结果下载说明.md)。原始 MAT、离线缓存、论文 Word 与参考资料继续保存在本地。

用户要求图片不生成 PDF，因此已删除本轮九份图表 PDF，当前绘图和导出核查只使用 SVG 与 600 dpi PNG。保留的十八份 PNG/SVG 与原文件哈希一致；两种绘图入口的导出短验证、九组实际图的格式核查及 13 项统计测试通过。历史执行快照与验收报告记录当时格式，保留用于追溯。
