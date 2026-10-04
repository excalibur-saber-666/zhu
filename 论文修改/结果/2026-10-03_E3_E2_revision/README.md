# E3 / E2 返修实验材料索引

本目录保存本轮新实验，与旧结果分开。完成状态以 `validation/delivery_audit.json`、`tables/E3/validation.json`、`tables/E2/validation.json` 为准；只有三者均通过才表示交付完成。

优先阅读：

1. [E3_E2_实验结果报告.md](E3_E2_实验结果报告.md)：实验定义、实际发现、所有主表、失败解释、适用边界。
2. [全部Follower定位指标.md](全部Follower定位指标.md)：所有场景、所有 follower、全部方法的 Maximum/Mean/CDF95/3D RMSE。
3. [图表与复算索引.md](图表与复算索引.md)：六组图、源数据、图注与校验记录。
4. 项目 `论文/修订记录/E3_E2_实验结果与修订建议.md`：面向论文返修的结果素材，未改正式稿。

## 文件位置

| 目录/文件 | 内容 |
| --- | --- |
| frozen_manifest.json | 正式运行前冻结的 seed、场景、参数、端点及统计定义 |
| raw/<scenario>/seed_XXXX.mat | 完整配置、噪声缓存、真值、故障段、四/两方法误差与逐更新日志 |
| derived/<scenario>/ | 离线状态 NPZ 与统计 JSON，可从 raw 重建 |
| tables/E3/ | 逐事件、逐 seed 延迟和健康统计，失败诊断 |
| tables/E2/ | 三层定位指标、配对差值、事件与健康统计、几何与缺失清单 |
| tables/figure_sources/ | 图的直接源数据；E2 曲线直接使用 E2/position_summary.csv |
| tables/raw_manifest.csv | 300 个原始文件的大小与 SHA256 |
| tables/fault_exposure.csv | 各场景、各 follower 的故障 edge-epoch 暴露 |
| tables/range_dynamics.csv | 每条 leader 边的真实距离变化与差分动态 |
| tables/admitted_constraints_per_seed.csv | 实际准入 leader 边数，区别于名义几何 |
| figures/ | PDF、SVG、600 dpi PNG |
| validation/ | 正式运行日志、回归测试、手算统计测试、图表及交付审计 |
| provenance/before/ | 修改前源文件快照；不用于正式新场景复算 |
| provenance/executed_source/ | 正式运行 MATLAB 与数据的实际快照 |
| provenance/analysis_source/ | 交付时 Python 统计/绘图/报告源代码快照 |
| provenance/effective_config_*.json | 各场景 seed 1 的生效参数摘录；全配置在各 MAT 中 |

`baseline_3f` 同时服务 E3、E2a、E2b；不要把三个用途累计成 150 个独立基准运行。唯一场景共六种，各 50 个 seed。

## 从项目根目录复算

MATLAB 新运行或断点续跑：

```matlab
addpath(fullfile(pwd,'code')); setup_project
run_revision_experiments('E3',1:50)
% 先完成下方 Python E3 分析，形成 E3_VALIDATED.json，再执行：
run_revision_experiments('E2',1:50)
```

已有检查点会核对配置后复用，不覆盖不匹配的正式结果。要重新运行完整实验，请在另一个明确的输出目录先复制冻结清单和执行源代码，并通过第三个参数传入目录；不要删除本批原始证据来“重新开始”。正式调试 seed 1001/1002 仅用于测试，不纳入 1:50。

Python 使用 `code/revision_python_requirements.txt` 中的依赖；本机依赖已在 `D:/codex_agent/工具/2026-10-03_E3_E2_revision/python_packages`。分析脚本会优先读取该目录；其他环境可设置进程级 `REVISION_PYTHON_DEPS`，或在正常 Python 环境安装这些依赖。不要更改全局 CODEX_HOME/TEMP/TMP。

```powershell
python -B -X utf8 code/analyze_revision_experiments.py --stage E3
python -B -X utf8 code/diagnose_revision_e3.py
python -B -X utf8 code/analyze_revision_experiments.py --stage E2
python -B -X utf8 code/audit_revision_delivery.py
python -B -X utf8 code/plot_revision_experiments.py --stage E3
python -B -X utf8 code/plot_revision_experiments.py --stage E2
python -B -X utf8 code/write_revision_report.py
```

分析 CLI 支持 `--output` 指定另一原始结果目录。诊断、审计与报告模块可从 Python 调用 `main(Path(...))` / `audit(Path(...))`；默认指向本目录。`--allow-partial` 仅用于运行途中核查，不能作为完整交付验收。

相关验证：

```matlab
test_revision_experiments
test_stage1_cusum_components
test_stage1_four_method_comparison_smoke
```

```powershell
python -B -X utf8 code/test_revision_analysis.py
```

本机 MATLAB 启动使用 `D:/MATLAB_R2025b/bin/matlab.exe -batch`，4 个隐藏进程按 seed 分片，每个 `maxNumCompThreads(1)`。启动脚本与进程日志在 `D:/codex_agent/工具/2026-10-03_E3_E2_revision`、`D:/codex_agent/记录/2026-10-03_E3_E2_revision`；正式运行记录同时保存在本目录 `validation/`。不会自动联网或推送 GitHub。

## 版本说明

原工作区有用户尚未提交的修改，Git HEAD 不是本轮实际执行代码的完整版本。`before_worktree.patch`、实际执行快照与其哈希共同说明来源。正式复算应使用 `provenance/executed_source/` 中的 code 与 data，及 `analysis_source/` 中的分析代码；不要只检出 HEAD 后假定与本批结果一致。论文原稿保持原哈希。根目录修稿方案已按后续授权更新，其原始版本保存在 `provenance/before/`；后续 E1/E4/E5 活动源码变更由新的阶段快照记录。

本目录的大型 raw/derived 和论文材料沿用项目忽略规则，未强行纳入 Git。辅助安装文件保留在 D 盘分类目录，正式数据不会仅存于临时目录。
