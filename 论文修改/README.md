# 网页版 GPT 审查入口：论文返修代码与实验结果

更新日期：2026-10-04。此文件夹专门整理本轮论文返修的源码、实验结果及审查说明。用途：让审查者能追踪本轮改动、核对结果与实现，并指出尚未解决的问题。先审查并给出建议，暂不执行算法修改或重跑实验。

## 1. 从哪里开始

- 仓库：[excalibur-saber-666/zhu](https://github.com/excalibur-saber-666/zhu)。
- 工作分支：[codex/random-nlos-gmm](https://github.com/excalibur-saber-666/zhu/tree/codex/random-nlos-gmm)。
- 集中审查目录：[论文修改/](https://github.com/excalibur-saber-666/zhu/tree/codex/random-nlos-gmm/%E8%AE%BA%E6%96%87%E4%BF%AE%E6%94%B9)。仓库根目录 code/ 保留唯一有效源码，本目录结果/为精简实验证据，审查差异/为提交补丁，论文/修订记录/为写作建议，历史方案/为旧执行方案；[优化讨论与建议](优化讨论与建议.md) 单列尚未实施的优化方案。
- [审稿问题与实验证据对照](审稿问题与实验证据对照.md)：12 条意见各有独立说明，依次列出原文、对应实验、可支持的回答、数据/图/源码链接及尚未完成的工作。
- [源码编码说明](源码编码说明.md)：部分旧基础 MATLAB 文件不是 UTF-8，另提供 `源码UTF8阅读副本/` 便于查看中文注释；原始源码和阶段快照仍按原字节保留。这些副本只改变文本编码，不代表新算法或新实验。
- 本次审查固定的源码版本：[742ab7a505bf00471a594be89302c947b91e5626](https://github.com/excalibur-saber-666/zhu/tree/742ab7a505bf00471a594be89302c947b91e5626)。后续说明文档提交不改变这个算法版本。
- 报告、表格与图：[实验 Release](https://github.com/excalibur-saber-666/zhu/releases/tag/revision-experiments-2026-10-03)。
- 推荐下载并提供给审查者：[代码与结果审查包 revision-gpt-review-2026-10-04.zip](https://github.com/excalibur-saber-666/zhu/releases/download/revision-experiments-2026-10-03/revision-gpt-review-2026-10-04.zip)。该包包含既有结果审阅材料、当前发布源码、提交差异、两阶段执行源码与配置快照、本说明和文件校验清单。

仓库网页、Release 附件与本地文件是不同的访问入口。审查开始时，应先列出实际成功读取的文件；不能以看到目录、文件名或链接代替读取正文。若无法读取附件，可以下载后提供给审查者；仍无法读取的材料应列为缺失。

GitHub 文件夹展示精简材料，完整固定源码、两阶段执行/分析快照及更多紧凑统计表只随审查 ZIP 提供。ZIP 的 code/ 与论文修改/是并列目录，问题说明里的源码链接可直接对应这份源码。大体积原始数据不在 ZIP 中。

## 2. 本轮到底改了什么

| 改动 | 对应文件 | 核查重点 |
| --- | --- | --- |
| 段级随机 NLOS | `stage1_cusum_3f3l_config.m`、`stage1_cusum_default_config.m`、`run_stage1_cusum_comparison.m` | 保留 13 个故障时段；每段抽一个正偏置，段内保持；双分量高斯混合、截断范围和独立随机流；同 seed 方法输入配对 |
| E3 隔离与恢复、健康机动；E2 规模和强度 | `revision_experiment_config.m`、`revision_relative_motion.m`、`revision_validate_config.m`、`run_revision_experiments.m` | 2/3/5 follower 的身份映射与几何；无 NLOS 场景确实改变相对距离；规模与强度分开研究 |
| E1 敏感性、E4 耗时、E5 消融 | `remaining_revision_config.m`、`run_remaining_revision_experiments.m`、`run_stage1_cusum_comparison.m` | 3×3 h–κ 网格只作敏感性；计时包含关系；消融只关闭目标机制 |
| 在线状态记录 | `compute_ekf_cusum_range_weights.m`、`run_stage1_cusum_comparison.m` | 记录预测值、冻结、报警、新因子准入及图迭代；本轮没有重新设计预测器的冻结/解除公式 |
| 全 follower 与 formation 统计、事件统计 | `analyze_revision_experiments.py`、`analyze_remaining_revision.py`、`compute_li_style_segment_performance.m` | 先按 seed 计算，再汇总；pooled 和 follower 等权 macro 并列；删失、失败、漏检的分母和终点 |
| 图、报告和审计 | `plot_*revision*.py`、`write_*revision*report.py`、`audit_*revision*.py`、对应测试 | 报告与 CSV 一致；负面结果保留；按用户要求只导出 PNG/SVG |

上述文件均位于仓库根目录 code/；本目录不重复放置活动源码。完整说明见 [返修实验实现与验证](../code/说明/2026-10-03_返修实验实现与验证.md)。

提交阅读顺序：

1. [22b92d7：实验实现](https://github.com/excalibur-saber-666/zhu/commit/22b92d7c8c64bcedd3982acd57aadee6defe187e)，28 个文件的主要实验改动。
2. [8bbe62c：验收与上传说明](https://github.com/excalibur-saber-666/zhu/commit/8bbe62c)，整理文档。
3. [742ab7a：取消图片 PDF 并发布审阅材料](https://github.com/excalibur-saber-666/zhu/commit/742ab7a505bf00471a594be89302c947b91e5626)，图表格式与发布说明。

审查包的 `审查差异/` 提供上述提交的补丁。每个补丁相对该提交的父提交；不要把 22b92d7 之前已有的冻结/恢复策略误写成本轮新引入的修改。当前源码使用 Git 中的版本；实际实验来源在下载包内另由各阶段 `provenance/executed_source/` 和 `analysis_source/` 快照标明。

## 3. 已完成与尚未完成

- E3→E2→E1→E4→E5 正式实验已完成。1230 个新唯一检查点、2120 次正式方法运行；中心点/A0 复用不增加独立重复。运行完成和文件验收通过，不等于算法没有误判。
- 主配置仍为 h=5、κ=0.5。现有论文路线、Kalman 更新与图优化数值公式未重新设计；新增实验通过配置显式开启。IMU 预积分及历史 tuned/recovered 配置不属于这批主结果。
- 已有 MATLAB 开发测试及 13 个 Python 统计测试通过，执行记录在报告中。本次只整理审查材料，未重新执行这些仿真或测试。
- 正式 Word 正文尚未修改，也未放入公开包。论文贡献、相关工作、术语、参考文献和逐条审稿回复仍需要后续写作与核查。不能仅凭代码包声称已完成全部文字返修。
- 2026-10-04 讨论的“降低健康误隔离、加快恢复”仍处于原因分析阶段，尚无优化代码或新结果。

## 4. 优先审查的问题

### A. E3 健康误隔离与恢复超时：已观察到

基准场景有 650 个注入故障，1 个未被实际隔离；649 个曾隔离事件中，629 个在固定 30 s 内确认稳定重新准入，20 个未满足该终点。合并失败率为 20/649；按 seed 平均失败率是另一口径，须分别标示。

无 NLOS 相对机动场景，16/50 次运行曾发生误隔离，其中 15 次为 F2–L3、1 次为 F2–L2；4 次持续到 600 s。健康边隔离占用率约 0.798%，不能用这一较小比例掩盖 32% 的运行发生过误隔离。

检查 `compute_ekf_cusum_range_weights.m` 的 `local_predict_range`、`local_update_range_predictor`、`local_update_alarm`，以及 `run_stage1_cusum_comparison.m` 的 `local_window_admission_mask`：

- 距离预测使用 alpha–beta 距离/距离变化率模型。
- CUSUM 达到 1 即停止绝对距离值的创新修正；报警门槛为 5，确认 2 个历元。冻结期间仍允许受门控的距离变化率更新，不能称为整个预测器停止运动。
- 解除条件包括 CUSUM 降到 1.2 且创新满足门控，或归一化创新绝对值连续 3 个历元不超过 1。
- 每个 follower 只隔离已报警 leader 边中 CUSUM 最大的一条；旧报警可能与新故障竞争。

健康 seed 2/F2–L3 的既有轨迹：313 s 首次进入本次持续冻结，预测距离偏差约 −0.65 m；324 s 确认报警并误隔离，偏差约 −1.33 m，此后持续到仿真结束。恢复 seed 5/event 2/F2–L2：故障在 135 s 结束，稳定重新准入延迟 111 s；结束后前 30 s 的预测偏差中位数约 −0.87 m。

这些日志和源码支持“正常动态预测偏差、提前冻结与恢复门控耦合”的解释，但尚未通过针对性机制对照证明因果。审查包仅提供已有诊断 CSV，未公开这些逐历元 NPZ/MAT；不要声称独立复查了完整时间轨迹。

相关文件：`结果/2026-10-03_E3_E2_revision/E3_E2_实验结果报告.md`、`tables/E3/healthy_isolation_episodes.csv`、`recovery_failure_diagnostics.csv`、`missed_isolation_diagnostics.csv`。恢复终点度量的是实际新因子稳定准入；不一定等同于报警解除。旧因子自然退出也不等同于新因子准入恢复。

### B. 统计与实验边界：需要核查实现

- E1：h∈{3,5,7}、κ∈{0.25,0.5,0.75}；敏感性点出现更好结果不代表已完成独立选参。若重新选参，calibration/evaluation seeds 必须分离。
- E2：不同规模同时改变节点构成、约束冗余及噪声维度。须检查 source node 映射、有效通信边与几何可观性；并列 pooled 与等权 macro，避免时序历元伪重复。
- E3：事件分母、未隔离、预先隔离、删失、30 s 终点与三个健康准入历元的边界应逐一核查。同一 follower 同时多条严重异常边未经充分验证。
- E4：分别核对 algorithm-specific 与 end-to-end online update 的起止位置。后者含每周期 50 次 SINS 传播及 1 Hz 状态更新；不含模拟传感器数据生成、日志落盘、绘图。最大观测完整周期 83.793 ms、超过 1 s 为 0/72000，只支持当前硬件/配置下的在线计算可行性。
- E4 的四方法耗时比较包含现有方法完整实现的差异，不能把所有差值归为 CUSUM 的额外开销；不另行重设计基线。
- E5：完整、无确认隔离、无软降权的 macro RMSE 均值分别为 1.065、1.484、1.104 m，仅覆盖基准场景。需核查关闭机制后检测状态仍正常执行，不能把两种机制的收益解释为可独立相加。
- 在线检测必须不读真值、故障时段、故障标签或注入幅值；离线注入和评估使用这些信息是允许的。测距权重按 sqrt(weight)/sigma_dis 只施加一次。

### C. 文档和材料边界

历史报告中的“未推送”“PDF/SVG/PNG”记录实验验收时状态。之后已授权发布；图片 PDF 已删除，现行图表只提供 PNG/SVG。审查时以本入口和现行下载说明解释时间先后，不把旧措辞当作新运行事实。

当前包支持源码审查、汇总结果核对和问题定位，不包含约 18 GB 原始/派生实验数据，也不包含 Word 和参考论文全文。完整独立复算和全文返修审查需要补充对应材料。

按用户指定范围，不扩展为既有 FGO 对照定义重设计，也不核查旧英文表 2 与历史随机 NLOS 表的来源差异。仍应指出本轮新增代码中有证据支持的统计或实现问题。

## 5. 阅读顺序

1. 本文件、`审稿问题与实验证据对照.md`、`优化讨论与建议.md`，再按具体问题进入 `审稿问题/` 对应目录。
2. `实验结果总索引.md`、`code/说明/2026-10-03_返修实验实现与验证.md`。
3. `审查差异/22b92d7.patch` 和 `code/revision_experiment_config.m`、`remaining_revision_config.m`。
4. `code/run_stage1_cusum_comparison.m`、`compute_ekf_cusum_range_weights.m`；需要时追踪 EKF 和图优化调用。
5. `code/analyze_revision_experiments.py`、`analyze_remaining_revision.py` 和相应统计测试。
6. 两阶段报告、E1/E4/E5 子报告、全部 follower/formation 表，以及相关汇总和诊断 CSV。
7. 对结果与源码版本的疑问，检查阶段执行快照、配置及清单。阶段快照与当前发布源码不能静默混用；Git 文本换行形式也可能与执行快照不同。

## 6. 可直接交给 GPT 的审查要求

> 请审查附件或 GitHub `论文修改/` 文件夹中的论文返修代码和实验材料，先阅读该目录的 README.md 和《优化讨论与建议.md》。暂时只给审查意见，不修改算法、不运行全部实验。
>
> 请先列出实际读取成功的关键源码、报告和 CSV，并列出缺失或无法读取的材料。固定审查源码版本为 742ab7a505bf00471a594be89302c947b91e5626；本轮主要实现差异见 22b92d7.patch，阶段原始实现见 provenance 快照。
>
> 优先检查：健康机动下误报警/误隔离、冻结与恢复耦合、旧报警竞争、随机输入配对与真值泄漏、规模映射与可观性、seed 级统计与 macro、事件恢复终点、两种计时边界、消融开关及权重是否重复施加。区分本轮新改动与既有算法限制。
>
> 每条意见请给出严重程度、文件/函数/行号或 CSV 行的依据、具体触发条件、影响和建议验证方式；明确区分“已观察到的问题”“源码支持的风险”“尚待验证的假设”。没有看到原始轨迹时，不声称已经独立复算；测试运行完成不能证明算法没有误判。
>
> 如建议优化，保持 baseline 并以配置启用新变体；在线逻辑禁止使用真值和注入标签。先提出最小改动与独立验证方案，同时检查误隔离、恢复延迟、漏检和定位误差。不要从正式评估集重新寻优，不扩大为与审稿意见无关的大规模实验。
