# Review296-1：参数依据与敏感性

## 审稿意见原文

> The paper should clarify how the CUSUM threshold and drift parameters are selected and how sensitive the performance is to these choices.

## 对应实验与处理状态

- 对应实验/工作：E1。
- 当前状态：PARTIAL：实验与汇总已完成，参数解释尚待写入正文。
- 正式回复准备状态：draft_with_placeholders；未生成正文最终页码/行号，尚不可直接作为提交完成证明。

## 现有证据可支持的回答

已完成 h∈{3,5,7}、κ∈{0.25,0.5,0.75} 的 3×3 网格，基准 NLOS 和无 NLOS 相对机动分别采用 seed 1–50；中心点复用既有结果，不增加独立重复。九点基准 macro RMSE 均值为 1.046–1.104 m，条件检测延迟均值为 0.185–1.718 s，健康误隔离随 κ 明显变化。

可回答为：当前参数的意义是误报警、响应速度与定位性能的折中。本轮保留 h=5、κ=0.5，分析局部敏感性，不用正式评估结果重新寻优，也不声称中心最优。κ=0.75 在本次健康场景没有观察到误隔离，仅是有限场景观测。

## 数据、图表和源码支撑

- [E1_参数敏感性结果.md](../../../结果/2026-10-03_E1_E4_E5_revision/E1_参数敏感性结果.md)
- [position_summary.csv](../../../结果/2026-10-03_E1_E4_E5_revision/tables/E1/position_summary.csv)
- [health_summary.csv](../../../结果/2026-10-03_E1_E4_E5_revision/tables/E1/health_summary.csv)
- [event_summary.csv](../../../结果/2026-10-03_E1_E4_E5_revision/tables/E1/event_summary.csv)
- [metrics_per_seed.csv](../../../结果/2026-10-03_E1_E4_E5_revision/tables/E1/metrics_per_seed.csv)
- [E1_parameter_sensitivity.svg](../../../结果/2026-10-03_E1_E4_E5_revision/figures/E1_parameter_sensitivity.svg)
- [remaining_revision_config.m](../../../../code/remaining_revision_config.m)
- [analyze_remaining_revision.py](../../../../code/analyze_remaining_revision.py)

## 正文需要落实的位置

Methods：h、κ 定义与选择解释；Results：敏感性；Discussion：折中及局部网格边界。正文页行号未产生。

## 尚未解决的问题和核查边界

没有独立参数标定实验；若改变主参数须另设 calibration/evaluation seeds，不能将当前九点分析写成理论最优或全局最优。

## 中文核对

本项区分已有实验/材料与待完成的文字修改；没有虚构论文页码、文献或优化结果。数据指标仍需由审查者与链接中的 CSV 和源码逐项核对。

[返回问题总表](../../../审稿问题与实验证据对照.md) · [返回审查入口](../../../README.md)
