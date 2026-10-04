# E1 / E4 / E5 实验与复算说明

本目录保存用户授权的剩余实验，顺序为 E1 参数敏感性 → E4 四方法计算与收敛 → E5 基准机制消融 → 整体验收。原稿与旧结果保留，主参数 h=5、κ=0.5；没有评估集重新选参。最终完成状态以各 `*_VALIDATED.json` 和 `validation/delivery_audit.json` 为准。

## 文件索引

| 内容 | 文件或目录 |
| --- | --- |
| 总体实验报告 | 论文返修实验整体验收报告.md |
| E1 九点与两个场景 | E1_参数敏感性结果.md、tables/E1/ |
| E4 两种计时、三阶段、收敛 | E4_计算性能与收敛结果.md、tables/E4/ |
| E5 A0 / A1 / A2 | E5_机制消融结果.md、tables/E5/ |
| 全部 follower / pooled / macro | 全部Follower与Formation指标.md |
| 预先冻结设计 | frozen_manifest.json |
| MATLAB 原始记录 | raw/阶段/网格点或变体/场景/seed_XXXX.mat |
| 基准复用说明 | 同目录 seed_XXXX.reuse.json，指向 E3/E2 原始记录 |
| 可重建的离线状态和统计 | derived/ |
| PDF / SVG / 600 dpi PNG | figures/ |
| 修改前、实际 MATLAB 与数据源快照 | provenance/before/、provenance/executed_source/ |
| Python 分析、绘图、报告快照 | provenance/analysis_source/ |
| 来源哈希、硬件、审计与日志 | provenance/、validation/、各阶段 source_manifest.csv |
| 图注、源表与 QA | 图表与复算索引.md |

## 统计和状态定义

各 seed 先计算各 follower 的三维 ENU Maximum、Mean、Hazen CDF95、RMSE，再汇总均值与样本 SD。定位覆盖 0–600 s、每 follower 30001 历元。macro 是各 follower 指标的等权平均；pooled 是合并三维误差范数后计算。macro CDF95 不是总体分位数，macro Maximum 不是总体最坏误差；等权也不消除不同规模的角色、几何或故障暴露差异。

健康统计从 16 s 起，与 E3 一致；相对机动阶段 260–460 s。区分新报警、新实际误隔离与健康历元占用。阈值越过、确认报警、新因子不准入与稳定恢复分别记录。每点事件标签和注入幅值只作离线统计，未用于在线决策。恢复要求连续 3 次准入且确认在固定 30 s 观察期内；未隔离不适用、漏检、预先状态、失败和删失分别保留。

E4 的每 seed 全时段 600 次更新，填窗 1–9 s、稳定 10–600 s（591 次）。Algorithm-specific 是方法处理部分；end-to-end 是 50 次 SINS 的计算之和加一次完整在线协同更新。排除离线模拟、非必要记录、绘图和导出，不包含通信或传感器等待。`GraphSolveSeconds` 为历史代码中的分支局部耗时，在 EKF 路径是测距更新部分，不解释为 EKF 的“图求解”。两种主要计时各保留 Mean/Median/P95/P99/Max 与 >1 s 比例。先在 seed 内计算，再跨 10 seed 汇总；绝对最大值另外报告。

## 复算已有记录

在项目根目录调用下面的命令。Python 3.12、NumPy 2.5.3、SciPy 1.18.1、Matplotlib 3.11.2 与 pypdf/Pillow 可用；版本记录和 requirements 随分析快照提供。可用进程变量 `REVISION_PYTHON_DEPS` 指定依赖目录，默认使用 D:/codex_agent/工具/2026-10-03_E3_E2_revision/python_packages。不要修改全局 TEMP、TMP 或 CODEX_HOME。

```powershell
python -B -X utf8 code/analyze_remaining_revision.py --stage E1
python -B -X utf8 code/analyze_remaining_revision.py --stage E4
python -B -X utf8 code/analyze_remaining_revision.py --stage E5
python -B -X utf8 code/plot_remaining_revision.py --stage E1
python -B -X utf8 code/plot_remaining_revision.py --stage E4
python -B -X utf8 code/plot_remaining_revision.py --stage E5
python -B -X utf8 code/audit_remaining_revision.py
python -B -X utf8 code/write_remaining_revision_report.py
```

完整统计器拒绝缺失、失败或 partial 记录；`--allow-partial` 仅用于运行中的监测，不写验收门。复用依赖 E3/E2 的 raw 和 derived，勿把 reuse.json 当作独立重复。

## MATLAB 重跑与来源恢复

在独立的项目副本中恢复本阶段 `provenance/executed_source/` 的 code/data，并将 `analysis_source/` 的 Python 文件放回副本 code。保留两个阶段的资料、原始记录及相同结果目录结构，解析复用和输入配对时需要 E3/E2 的 raw/derived。E3/E2 单独重跑应使用其自己的执行快照。本轮开始时已有未提交修改，Git HEAD 不能独自代表实际实验版本。

```matlab
addpath(fullfile(pwd,'code')); setup_project
run_remaining_revision_experiments('E1',1:50)
% 离线完整验证 E1 后，才允许 E4；单进程、单计算线程
run_remaining_revision_experiments('E4',1:10)
% 离线完整验证 E4 后，才允许 E5
run_remaining_revision_experiments('E5',1:50)
```

运行器检查已有文件是否与冻结配置一致，并保存失败记录。E1 与 E5 可按互不重叠 seed 分片；正式 E4 禁止并行计时。同一主机重跑的耗时不会逐值相等，E4 的定位结果则须与 E2 同 seed 对应方法一致。

辅助工具、缓存、运行器与预览在 D:/codex_agent 对应任务子目录。正式材料在本项目，未推送 GitHub，未修改论文 Word。
