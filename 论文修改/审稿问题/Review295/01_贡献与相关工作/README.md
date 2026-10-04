# Review295-1：贡献与相关工作

## 审稿意见原文

> The contribution of this paper is not clear to multi-UAV cooperative navigation. More details should be given to stress the main contributions of this paper, compared with existing work in literature,

## 对应实验与处理状态

- 对应实验/工作：E5（增强证据）、E2（场景支撑）。
- 当前状态：PARTIAL：已有机制与场景证据，贡献段和最相关工作对比尚待写入正文。
- 正式回复准备状态：draft_with_placeholders；未生成正文最终页码/行号，尚不可直接作为提交完成证明。

## 现有证据可支持的回答

目前可支持的回答：方法在逐边时序检测、软降权、确认隔离、恢复与滑窗定位之间形成完整处理流程；基准 E5 对照支持两种异常处理机制在当前实现中的作用。完整、无隔离、无软权重的 macro RMSE 为 1.065、1.484、1.104 m，50 个配对种子中两个消融变体均劣于完整方法。

这些实验不能单独证明文献创新性。应增加最相关工作的直接对比，逐项说明已有工作是否具备相应机制。CUSUM、alpha–beta、FGO 和 Kalman filter 本身不独立宣称原创；FGO 到 Kalman 的反馈也不单列原创贡献。

## 数据、图表和源码支撑

- [EXPERIMENT_CONTEXT.md](../../../../code/EXPERIMENT_CONTEXT.md)
- [E5_机制消融结果.md](../../../结果/2026-10-03_E1_E4_E5_revision/E5_机制消融结果.md)
- [position_summary.csv](../../../结果/2026-10-03_E1_E4_E5_revision/tables/E5/position_summary.csv)
- [paired_center_summary.csv](../../../结果/2026-10-03_E1_E4_E5_revision/tables/E5/paired_center_summary.csv)
- [E5_mechanism_ablation.svg](../../../结果/2026-10-03_E1_E4_E5_revision/figures/E5_mechanism_ablation.svg)

## 正文需要落实的位置

Introduction：三条贡献及相关工作对比表/段落；Discussion：适用边界。正文未修订，页码/行号待最终稿产生。

## 尚未解决的问题和核查边界

最相关文献核对和对比尚未完成，不能把 RMSE 改善当作算法原创性证明。

## 中文核对

本项区分已有实验/材料与待完成的文字修改；没有虚构论文页码、文献或优化结果。数据指标仍需由审查者与链接中的 CSV 和源码逐项核对。

[返回问题总表](../../../审稿问题与实验证据对照.md) · [返回审查入口](../../../README.md)
