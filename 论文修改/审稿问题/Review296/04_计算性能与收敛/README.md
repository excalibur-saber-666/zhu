# Review296-4：计算性能与收敛

## 审稿意见原文

> The comparison focuses mainly on positioning error metrics, while other relevant aspects such as computational cost, convergence behavior, and real-time feasibility are not reported.

## 对应实验与处理状态

- 对应实验/工作：E4。
- 当前状态：PARTIAL：两种计时、尾部和迭代统计已完成，正文尚待补充。
- 正式回复准备状态：draft_with_placeholders；未生成正文最终页码/行号，尚不可直接作为提交完成证明。

## 现有证据可支持的回答

在 2F3L/3F3L/5F3L 各 seed 1–10，串行、单计算线程，对 EKF/FGO/CUSUM-EKF/CUSUM-FGO 测量。报告 algorithm-specific 与 end-to-end online update 两种耗时，后者包括每周期 50 次 SINS 传播及 1 Hz 更新，至状态反馈完成；模拟输入生成、文件输出和绘图不计入。

全部观测完整周期最大 83.793 ms，超过 1 s 为 0/72000；图优化未收敛和触顶统计见表。可回答为当前硬件/实现与 1 Hz 配置下的在线计算可行性，不能扩大为机载硬实时保证。

## 数据、图表和源码支撑

- [E4_计算性能与收敛结果.md](../../../结果/2026-10-03_E1_E4_E5_revision/E4_计算性能与收敛结果.md)
- [runtime_summary.csv](../../../结果/2026-10-03_E1_E4_E5_revision/tables/E4/runtime_summary.csv)
- [timing_seeds_per_seed.csv](../../../结果/2026-10-03_E1_E4_E5_revision/tables/E4/timing_seeds_per_seed.csv)
- [E4_runtime_convergence.svg](../../../结果/2026-10-03_E1_E4_E5_revision/figures/E4_runtime_convergence.svg)
- [run_stage1_cusum_comparison.m](../../../../code/run_stage1_cusum_comparison.m)
- [analyze_remaining_revision.py](../../../../code/analyze_remaining_revision.py)
- [test_remaining_revision_experiments.m](../../../../code/test_remaining_revision_experiments.m)

## 正文需要落实的位置

Methods：硬件与计时起止；Results：四方法均值、P95/P99、最大值和 GN 迭代；Discussion：实时性边界。页行号待修订。

## 尚未解决的问题和核查边界

现有完整方法实现的结构与计算差异不能全部归为 CUSUM 检测成本；未测试外部 I/O 延迟或机载硬实时。

## 中文核对

本项区分已有实验/材料与待完成的文字修改；没有虚构论文页码、文献或优化结果。数据指标仍需由审查者与链接中的 CSV 和源码逐项核对。

[返回问题总表](../../../审稿问题与实验证据对照.md) · [返回审查入口](../../../README.md)
