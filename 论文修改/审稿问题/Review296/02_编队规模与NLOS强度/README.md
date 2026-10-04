# Review296-2：编队规模与NLOS强度

## 审稿意见原文

> The authors should include additional scenarios with different formation sizes and varying NLOS intensity to strengthen the generality of the conclusions.

## 对应实验与处理状态

- 对应实验/工作：E2a 规模、E2b 强度。
- 当前状态：PARTIAL：实验和 follower/formation 统计已完成，正文尚待补充。
- 正式回复准备状态：draft_with_placeholders；未生成正文最终页码/行号，尚不可直接作为提交完成证明。

## 现有证据可支持的回答

分别研究 2F3L/3F3L/5F3L（固定基准 NLOS）与弱/基准/强 NLOS（固定 3F3L，偏置幅值 ×0.5/1/1.5）。各场景 seed 1–50，四种方法输入配对。CUSUM-FGO 的 follower 等权 macro RMSE 均值：规模为 1.494/1.065/0.855 m，强度为 1.179/1.065/1.071 m。

同时提供逐 follower、pooled 与等权 macro。规模比较需要结合几何和有效测距边解释；仅能支持指定场景范围的泛化。实验不是全部规模×全部强度的全因子扫描。

## 数据、图表和源码支撑

- [E3_E2_实验结果报告.md](../../../结果/2026-10-03_E3_E2_revision/E3_E2_实验结果报告.md)
- [position_summary.csv](../../../结果/2026-10-03_E3_E2_revision/tables/E2/position_summary.csv)
- [metrics_per_seed.csv](../../../结果/2026-10-03_E3_E2_revision/tables/E2/metrics_per_seed.csv)
- [geometry_per_seed.csv](../../../结果/2026-10-03_E3_E2_revision/tables/E2/geometry_per_seed.csv)
- [geometry_all_scenarios.csv](../../../结果/2026-10-03_E3_E2_revision/tables/geometry_all_scenarios.csv)
- [admitted_constraints_per_seed.csv](../../../结果/2026-10-03_E3_E2_revision/tables/admitted_constraints_per_seed.csv)
- [E2a_formation_size.svg](../../../结果/2026-10-03_E3_E2_revision/figures/E2a_formation_size.svg)
- [E2b_NLOS_intensity.svg](../../../结果/2026-10-03_E3_E2_revision/figures/E2b_NLOS_intensity.svg)
- [revision_experiment_config.m](../../../../code/revision_experiment_config.m)
- [revision_validate_config.m](../../../../code/revision_validate_config.m)

## 正文需要落实的位置

Results：分别增加规模与 NLOS 强度小节；Discussion：几何、冗余及节点构成变化。页行号待修订。

## 尚未解决的问题和核查边界

不同规模的节点构成、冗余与噪声维度不能完全分离；不声称任意规模或任意 NLOS 均鲁棒。

## 中文核对

本项区分已有实验/材料与待完成的文字修改；没有虚构论文页码、文献或优化结果。数据指标仍需由审查者与链接中的 CSV 和源码逐项核对。

[返回问题总表](../../../审稿问题与实验证据对照.md) · [返回审查入口](../../../README.md)
