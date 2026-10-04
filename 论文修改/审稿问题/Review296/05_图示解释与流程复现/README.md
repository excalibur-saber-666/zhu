# Review296-5：图示解释与流程复现

## 审稿意见原文

> Several figures are referenced without sufficient interpretation, and the reproduction of the method's pipeline is partly obscured by unclear formatting in the rendered text.

## 对应实验与处理状态

- 对应实验/工作：E3状态链、E1–E5结果解释；源码和配置快照。
- 当前状态：PARTIAL：可复现材料和结果图已整理，正文格式与流程图仍待落实。
- 正式回复准备状态：draft_with_placeholders；未生成正文最终页码/行号，尚不可直接作为提交完成证明。

## 现有证据可支持的回答

可根据 E3 状态链解释 range prediction→CUSUM→soft weighting→alarm confirmation→selective isolation/recovery→FGO→Kalman feedback；应明确新因子准入与窗口内旧因子保留的区别。按源码、配置、执行快照、种子、统计器与测试建立可追踪流程。

每张结果图需解释对应问题、指标含义、曲线/表格差异和负面案例。当前代码与精简结果可支持方法审查，完整独立复算仍需本地原始数据；尚未替换 Word 中的流程图或修复所有公式排版。

## 数据、图表和源码支撑

- [E3_state_chain.svg](../../../结果/2026-10-03_E3_E2_revision/figures/E3_state_chain.svg)
- [图表与复算索引.md](../../../结果/2026-10-03_E3_E2_revision/图表与复算索引.md)
- [图表与复算索引.md](../../../结果/2026-10-03_E1_E4_E5_revision/图表与复算索引.md)
- [README.md](../../../结果/2026-10-03_E3_E2_revision/README.md)
- [README.md](../../../结果/2026-10-03_E1_E4_E5_revision/README.md)
- [2026-10-03_返修实验实现与验证.md](../../../../code/说明/2026-10-03_返修实验实现与验证.md)
- [revision_experiment_config.m](../../../../code/revision_experiment_config.m)
- [remaining_revision_config.m](../../../../code/remaining_revision_config.m)

## 正文需要落实的位置

Methods：符号、公式和伪代码；Fig.1：确认、隔离、恢复；Results：逐图解释。实际改稿位置未产生。

## 尚未解决的问题和核查边界

当前包不包含完整 MAT/NPZ，不能声称已具备脱离本地原始数据的完整复算条件。

## 中文核对

本项区分已有实验/材料与待完成的文字修改；没有虚构论文页码、文献或优化结果。数据指标仍需由审查者与链接中的 CSV 和源码逐项核对。

[返回问题总表](../../../审稿问题与实验证据对照.md) · [返回审查入口](../../../README.md)
