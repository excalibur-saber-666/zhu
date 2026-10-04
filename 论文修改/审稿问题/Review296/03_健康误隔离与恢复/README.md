# Review296-3：健康误隔离与恢复

## 审稿意见原文

> The selective isolation mechanism after alarm confirmation is described but not analyzed in detail, particularly regarding the risk of isolating healthy edges or failing to recover them promptly.

## 对应实验与处理状态

- 对应实验/工作：E3（含无NLOS机动健康压力场景）。
- 当前状态：PARTIAL：诊断及指标已完成；已知失误保留，优化尚未实施。
- 正式回复准备状态：draft_with_placeholders；未生成正文最终页码/行号，尚不可直接作为提交完成证明。

## 现有证据可支持的回答

基准场景 650 个故障有 1 个未被实际隔离；649 个曾隔离事件中 629 个在固定 30 s 内确认稳定准入恢复、20 个未满足。20 个中 14 个在后续既有轨迹恢复，6 个到结束仍未恢复。健康机动 16/50 次运行有误隔离，4 次持续到 600 s。

可回答：已明确统计误报警、误隔离、检测/隔离/恢复延迟和恢复失败，并显示健康机动下确实存在风险。不能写成风险已消除。诊断指向提前冻结、预测参考偏移与解除门控耦合，但因果解释仍需对照验证；具体优化意见见专门文件。

## 数据、图表和源码支撑

- [E3_E2_实验结果报告.md](../../../结果/2026-10-03_E3_E2_revision/E3_E2_实验结果报告.md)
- [event_summary.csv](../../../结果/2026-10-03_E3_E2_revision/tables/E3/event_summary.csv)
- [events_per_seed.csv](../../../结果/2026-10-03_E3_E2_revision/tables/E3/events_per_seed.csv)
- [health_summary.csv](../../../结果/2026-10-03_E3_E2_revision/tables/E3/health_summary.csv)
- [health_per_seed.csv](../../../结果/2026-10-03_E3_E2_revision/tables/E3/health_per_seed.csv)
- [healthy_isolation_episodes.csv](../../../结果/2026-10-03_E3_E2_revision/tables/E3/healthy_isolation_episodes.csv)
- [recovery_failure_diagnostics.csv](../../../结果/2026-10-03_E3_E2_revision/tables/E3/recovery_failure_diagnostics.csv)
- [missed_isolation_diagnostics.csv](../../../结果/2026-10-03_E3_E2_revision/tables/E3/missed_isolation_diagnostics.csv)
- [E3_state_chain.svg](../../../结果/2026-10-03_E3_E2_revision/figures/E3_state_chain.svg)
- [E3_recovery_failure.svg](../../../结果/2026-10-03_E3_E2_revision/figures/E3_recovery_failure.svg)
- [E3_healthy_maneuver.svg](../../../结果/2026-10-03_E3_E2_revision/figures/E3_healthy_maneuver.svg)
- [优化讨论与建议.md](../../../优化讨论与建议.md)
- [compute_ekf_cusum_range_weights.m](../../../../code/compute_ekf_cusum_range_weights.m)
- [run_stage1_cusum_comparison.m](../../../../code/run_stage1_cusum_comparison.m)

## 正文需要落实的位置

Methods：状态与准入规则；Results：E3；Discussion：误隔离、恢复超时及每 follower 一条主要异常边的边界。页行号待修订。

## 尚未解决的问题和核查边界

稳定准入与报警清除不同；占用率和发生过误隔离的运行比例不同；多条同时严重异常边未充分验证。优化建议不是已完成结果。

## 中文核对

本项区分已有实验/材料与待完成的文字修改；没有虚构论文页码、文献或优化结果。数据指标仍需由审查者与链接中的 CSV 和源码逐项核对。

[返回问题总表](../../../审稿问题与实验证据对照.md) · [返回审查入口](../../../README.md)
