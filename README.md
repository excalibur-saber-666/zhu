# 多 UAV 协同导航与论文返修实验

MATLAB 项目，比较 EKF、FGO、CUSUM-EKF 和 CUSUM-FGO。当前论文基准为 3F3L、段级随机 NLOS；主参数 h=5、κ=0.5，图优化更新频率 1 Hz。

## 从哪里开始

- [论文修改与审查入口](论文修改/README.md)：源码版本、结果下载、已知问题及阅读顺序。
- [12 条审稿意见与实验证据对照](论文修改/审稿问题与实验证据对照.md)：逐条回答依据、实验、数据、图表、源码及待完成事项。
- [误隔离与恢复优化讨论](论文修改/优化讨论与建议.md)：原因分析及尚未实施的改进建议。
- [完整算法说明](code/EXPERIMENT_CONTEXT.md)：模型、方法边界和函数映射。

## 仓库目录

| 目录 | 内容 |
| --- | --- |
| `code/` | 唯一有效源码、配置、测试及初始位置输入 |
| `论文修改/` | 审稿问题、回答依据、精简结果、优化建议与历史方案 |
| `论文/修订记录/` | 保留的早期论文公式与代码说明 |

原始 MAT、逐历元缓存、Word 正文和参考论文在本地项目保留。GitHub 提供精简结果；固定源码及两阶段执行快照随审查 ZIP 下载，避免仓库重复堆放。

## MATLAB 入口

从本项目根目录执行：

```matlab
addpath(fullfile(pwd, 'code')); setup_project
cfg = stage1_cusum_3f3l_config('quick');
report = main_stage1('cusum_fgo', cfg);
```

短测试配置用于检查运行，不作为论文结果。正式实验配置见 `revision_experiment_config` 和 `remaining_revision_config`，执行入口为 `run_revision_experiments`、`run_remaining_revision_experiments`。

E1–E5 已完成，已观察到健康误隔离和恢复超时；正式 Word 及部分文字/引用返修尚未完成。在线检测不使用真值或故障标签，IMU 预积分和历史 tuned/recovered 配置不属于当前主结果。
