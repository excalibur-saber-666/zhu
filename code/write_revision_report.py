"""Build the Chinese delivery report from audited E3/E2 tables, without simulations."""
import csv
import json
from pathlib import Path
from collections import Counter
from analyze_revision_experiments import ROOT, DEFAULT_OUTPUT, METRICS

SCENARIOS=['size_2f','baseline_3f','size_5f','nlos_weak','nlos_strong']
SCENE={'size_2f':'2F3L / 基准 NLOS','baseline_3f':'3F3L / 基准 NLOS',
       'size_5f':'5F3L / 基准 NLOS','nlos_weak':'3F3L / 弱 NLOS（0.5×）',
       'nlos_strong':'3F3L / 强 NLOS（1.5×）','healthy_maneuver':'3F3L / 健康相对机动'}
METHOD={'ekf':'EKF','fgo':'FGO','cusum_ekf':'CUSUM-EKF','cusum_fgo':'CUSUM-FGO'}


def read(output,name):
    with (output/'tables'/name).open(encoding='utf-8-sig',newline='') as f:
        return list(csv.DictReader(f))


def tab(headers,rows):
    return '\n'.join(['| '+' | '.join(map(str,headers))+' |','| '+' | '.join(['---']*len(headers))+' |']+
                     ['| '+' | '.join(map(str,row))+' |' for row in rows])+'\n'


def ms(row,key,multiplier=1,digits=3):
    return f"{float(row[key+'_Mean'])*multiplier:.{digits}f} ± {float(row[key+'_SD'])*multiplier:.{digits}f}"


def main(output=DEFAULT_OUTPUT):
    audit=json.loads((output/'validation'/'delivery_audit.json').read_text(encoding='utf-8'))
    assert audit['passed'] and audit['unique_scenario_seed_runs']==300
    for stage in ('E3','E2'):
        assert json.loads((output/'tables'/stage/'validation.json').read_text())['passed']
    position=read(output,'E2/position_summary.csv'); healthy_position=read(output,'E3/position_summary.csv')
    lookup={(r['Scenario'],r['Method'],r['Aggregation'],int(r['Follower'])):r for r in position}
    events=[r for r in read(output,'E3/events_per_seed.csv') if r['Method']=='cusum_fgo']
    event_summary=next(r for r in read(output,'E3/event_summary.csv') if r['Scenario']=='baseline_3f' and r['Method']=='cusum_fgo')
    health=[r for r in read(output,'E3/health_summary.csv') if r['Method']=='cusum_fgo' and r['EdgeScope']=='leader']
    health_seed=[r for r in read(output,'E3/health_per_seed.csv') if r['Method']=='cusum_fgo' and r['EdgeScope']=='leader']
    geometry=read(output,'geometry_all_scenarios.csv'); exposure=read(output,'fault_exposure.csv')
    dynamics=read(output,'range_dynamics.csv'); paired=read(output,'E2/paired_summary.csv')
    failures=read(output,'E3/recovery_failure_diagnostics.csv'); healthy_episodes=read(output,'E3/healthy_isolation_episodes.csv')
    counts={label:Counter(e[label+'Status'] for e in events) for label in ('Detection','Alarm','Isolation','Recovery')}
    recovered=counts['Recovery']['recovered']; failed=counts['Recovery']['failed']
    healthy_run_count=len({e['Seed'] for e in healthy_episodes})
    position_table=lambda aggregation:tab(['场景','方法','Maximum (m)','Mean (m)','CDF95 (m)','3D RMSE (m)'],
        [[SCENE[s],METHOD[m]]+[ms(lookup[s,m,aggregation,0],key) for key in METRICS] for s in SCENARIOS for m in METHOD])
    geomrows=[]
    for s in SCENARIOS+['healthy_maneuver']:
        rows=[r for r in geometry if r['Scenario']==s]
        exp=[r for r in exposure if r['Scenario']==s and r['Window']=='full']
        geomrows.append([SCENE[s],len(exp),','.join(sorted({r['TotalRangeEdges'] for r in rows})),
            ','.join(sorted({r['Rank'] for r in rows})),f"{max(float(r['Condition']) for r in rows):.3f}",
            f"{max(float(r['MaxRange']) for r in rows):.3f}",
            f"{sum(int(r['FaultEdgeEpochs']) for r in exp)}/{sum(int(r['ObservedLeaderEdgeEpochs']) for r in exp)}",
            f"{100*sum(int(r['FaultEdgeEpochs']) for r in exp)/sum(int(r['ObservedLeaderEdgeEpochs']) for r in exp):.3f}%"])
    healthrows=[]
    for s,phase in [('baseline_3f','all'),('healthy_maneuver','all'),('healthy_maneuver','pre'),('healthy_maneuver','maneuver'),('healthy_maneuver','post')]:
        summary=next(r for r in health if r['Scenario']==s and r['Phase']==phase)
        rows=[r for r in health_seed if r['Scenario']==s and r['Phase']==phase]
        healthrows.append([SCENE[s]+' / '+phase,sum(int(r['HealthyEdgeEpochs']) for r in rows),
            sum(int(r['AlarmNewHealthyEpisodes']) for r in rows),sum(int(r['IsolationNewHealthyEpisodes']) for r in rows),
            ms(summary,'AlarmHealthyOccupancy',100),ms(summary,'IsolationHealthyOccupancy',100)])
    pairrows=[]
    for s in SCENARIOS:
        for m in ['ekf','fgo','cusum_ekf']:
            r=next(r for r in paired if r['Scenario']==s and r['Aggregation']=='macro' and r['Comparison']=='cusum_fgo_minus_'+m)
            baseline=float(lookup[s,m,'macro',0]['RMSE3D_Mean'])
            difference=float(r['RMSE3DDifference_Mean'])
            pairrows.append([SCENE[s],f"CUSUM-FGO − {METHOD[m]}",ms(r,'RMSE3DDifference'),f"{-100*difference/baseline:.1f}%",r['RMSE3DDifference_N']])
    healthy_dyn=[r for r in dynamics if r['Scenario']=='healthy_maneuver']
    report=f'''# E3 / E2 论文返修实验结果报告

日期：2026-10-03。范围：Review296 R2.3（选择性隔离、误隔离与恢复）及 R2.2（编队规模与 NLOS 强度）。

**本轮已完成 6 个唯一场景 × 50 个固定 seed，共 300 个场景检查点、1100 次方法仿真；正式运行失败 0、缺失 0。** E3 验收后才启动 E2；3F3L 基准在 E3、E2a、E2b 之间复用同一批原始记录。全部计算留在本地，未推送 GitHub，未改论文正式原稿。

主要发现：当前 CUSUM-FGO 在本次配置中的定位性能见下表；但不能声称异常隔离与恢复没有失败。基准场景 650 个故障中有 1 个未被实际隔离；649 个发生过隔离的事件中，629 个在 30 s 内确认稳定恢复、20 个未在该期限内确认恢复。健康相对机动场景中，{healthy_run_count}/50 个 seed 出现误报警并误隔离，4 个直到仿真结束仍未解除。

## 1. 实际实现、修改范围与参数

本阶段依据用户修订的执行提示词、Review296 和封存的执行代码实施。保留原有四方法的定义及参数；只增加默认关闭的实验场景能力、可复算日志、统计与绘图。本阶段范围为 E3/E2；随后授权的 E1 敏感性、E4 性能评估和 E5 消融在另一结果目录保存。

- 原运行器给所有 UAV 相同姿态和速度，共同转弯不能充分检验相对测距动态。新增独立 `healthy_maneuver`：260–460 s 叠加平滑相对偏航和体轴速度变化，偏航系数 4°、速度系数 0.4 m/s，F1/F2/F3 系数为 1/−1/0.5。保留原有共同转弯，按各自轨迹生成一致的理想 IMU。
- 原日志中的 `baseline_frozen` 不能充分表达预测器的实际更新状态。增加真实的距离水平冻结、水平校正、冻结前后速度与预测值记录；没有改变预测器或检测器公式。
- 新增明确的 follower 数据源映射、固定事件抽样索引、幅值缩放和四方法原始输入保存。旧默认分支保持数值一致，已有输出不覆盖。

主参数为 h=5、κ=0.5、λ=0.9，15 个校准历元、2 个报警确认历元、3 个解除历元；alpha=0.30、beta=0.05、预测标准差 0.80 m、测距标准差 0.20 m；窗口长度 10，IMU preintegration 关闭。每个场景的最终生效参数摘录位于 `provenance/effective_config_*.json`，完整配置保存在每个 MAT 的 `payload.cfg`。

### 实际状态语义

阈值越界、confirmed alarm 与实际停止新因子准入分别记录。选择性规则对每个 follower 仅重点处理当前最大 CUSUM 的 leader edge；报警并不必然等于该边已隔离。隔离后继续读取测距进行在线检测，但不将该边的新测距因子加入优化。已在窗口内的历史因子随窗口推进退出，不会在报警瞬间全部删除。

“冻结预测器”具体指不再用当前创新直接校正距离水平；满足速度差门限时，距离变化率仍可更新。因此不应写成预测器所有状态完全停止演化。解除要求连续 3 次满足低 CUSUM 且创新不过大，或满足更严格的正常创新门限。解除与实际重新准入均按日志核查，离线真值和故障标签不参与在线决策。

## 2. 冻结设计与统计口径

正式 seed 为 1–50；1001、1002 仅用于功能调试。没有按正式结果挑 seed 或改变 h、κ、机制与场景。每次仿真 600 s、积分步长 0.02 s、测距及图优化周期 1 s。

定位评价覆盖 0–600 s 的全部 30001 个历元，包含启动期；无缺失样本。以三维 ENU 误差范数 e 定义 Maximum=max(e)、Mean=mean(e)、CDF95=95% 分位数、3D RMSE=sqrt(mean(e²))。CDF95 使用 Hazen 分位数规则，与 MATLAB 默认 `prctile` 的中点经验分位约定一致；RMSE 已与原 MATLAB 输出逐 seed 核对。

每个 seed 先计算全部 follower 指标、pooled overall 以及等权 macro-average，再跨 50 个 seed 计算均值与样本标准差（ddof=1）。macro CDF95 是“各 follower 的 CDF95 的均值”，macro Maximum 是“各 follower 最大误差的均值”，均不是整体尾部分位或整体最坏误差。各 follower 样本数相同，所以两种 Mean 相等；其余指标仍可能不同。时序、边和事件均不作为独立 Monte Carlo 重复。本文报告描述性均值±SD及同 seed 配对差，不进行显著性或总体可靠性推断。

E3 排除前 15 个更新历元，健康统计覆盖 16–600 s。故障区间为测距历元上的闭区间 [start,end]。阈值检测、确认报警、实际隔离分别按区间内的首次上升沿匹配；故障前已有状态单列，不记为 0 s 成功响应。不存在同一边重叠故障。未检测、未隔离保留记录及缺失延迟。

恢复适用人群为故障期间确实被隔离过的事件，含单列的预先隔离事件。故障结束后连续 3 个可观测健康历元实际准入才算稳定恢复，时间戳记这 3 个历元中的第 1 个，但 3 个历元都必须处于预先固定的 30 s 观察期内。此指标度量实际准入恢复，不要求该边 CUSUM 及报警状态同时完全复位；最大边选择变化也可能使一条仍报警的边重新准入，因此报警解除须另看状态日志。恢复后再次隔离另记。若观察完整而未满足条件记失败；若下一同边故障、缺测或仿真结束提前截断记删失；从未隔离记不适用。失败率分母为恢复成功+失败，删失另报。所有定义已在正式运行前写入 `frozen_manifest.json`。

## 3. E3：检测、隔离与恢复

两种 CUSUM 方法使用同一独立测距预测器和相同参数；本次逐 seed 核查了创新、双侧证据、权重、报警、准入与冻结状态相同。因此下列检测统计同时适用于 CUSUM-EKF、CUSUM-FGO，其定位输出仍分别保留。

'''
    report+=tab(['状态','故障内新发生','故障前已存在','漏检/未隔离','不可观测'],
        [[name,counts[key]['detected'],counts[key]['preexisting'],counts[key]['missed'],counts[key]['unobservable']]
         for name,key in [('阈值检测','Detection'),('确认报警','Alarm'),('实际隔离','Isolation')]])
    report+='\n'+tab(['条件延迟','跨 seed 均值 ± SD (s)','成功事件 / 全部故障','有条件均值的 seed 数'],
        [[name,ms(event_summary,key+'Delay'),f"{counts[key]['detected']}/650",event_summary[key+'Delay_N']]
         for name,key in [('阈值检测','Detection'),('确认报警','Alarm'),('实际隔离','Isolation')]]+
        [['稳定恢复',ms(event_summary,'RecoveryDelay'),f'{recovered}/649 个曾隔离事件',event_summary['RecoveryDelay_N']]])
    report+=f'''
延迟先在每个 seed 内对该状态的成功事件取均值，再跨 seed 汇总。预先存在状态与失败事件未被填成零，故上表延迟是条件统计。

恢复：成功 {recovered}，30 s 内未确认恢复 {failed}，提前截断删失 {counts['Recovery']['censored']}，未发生隔离而不适用 {counts['Recovery']['not_applicable']}。每个 seed 的失败率再平均为 **{ms(event_summary,'RecoveryFailureRate',100)}%**；描述性合并比率为 {failed}/{recovered+failed}={100*failed/(recovered+failed):.3f}%，两种口径分别标示。成功稳定恢复后又在观察期内隔离的事件共 2 个。

### 健康边的误报警与实际误隔离

下表的新事件数是 50 个 seed 的描述性合计；占用率先按 seed 计算，再汇总均值±SD。分母是可观测、当前无故障的 leader–follower edge-epoch。1 Hz 下也对应 edge-second 暴露时长。基准场景的故障后遗留状态仍计入健康占用，不能排除以美化误报率。

'''
    report+=tab(['场景/阶段','健康 edge-epoch 合计','新误报警','新误隔离','报警占用率 %','隔离占用率 %'],healthrows)
    report+='''
`pre`=16–259 s，`maneuver`=260–460 s，`post`=461–600 s；`all` 是三者合并，不额外计一次。基准健康边报警占用 5332/252200，其中 4797 个历元的当前报警始于故障期；健康隔离占用 4890/252200，其中 2869 个历元的当前隔离始于故障期。新健康隔离可以来自已报警边在最大证据选择中重新成为被隔离边，因此新隔离数不必等于新报警数。逐 follower–follower 边的健康统计也保存在源表中。

健康机动场景的 16 次报警和 16 次隔离均发生于预定机动阶段，分布在 16/50 个 seed，启动期后的机动前阶段无误报警或误隔离。15 次发生于 F2–L3，1 次发生于 F2–L2；最早 322 s，最晚 420 s。12 次后来解除，4 次延续到 600 s。无 NLOS 场景的总健康隔离占用为 2100/263250=0.798%，事件率 16/73.125=0.219 次/edge-hour；小占用比例不能掩盖 32% 的运行曾出现误隔离。

'''
    report+=f"实际健康轨迹的最大 leader–follower 距离变化为 {max(float(r['RangeExcursion']) for r in healthy_dyn):.3f} m，最大绝对距离变化率 {max(float(r['MaxAbsRangeRate']) for r in healthy_dyn):.3f} m/s，1 s 差分诊断的最大绝对变化加速度 {max(float(r['MaxAbsRangeAcceleration']) for r in healthy_dyn):.3f} m/s²。记录覆盖所有 9 条 leader 边，证明这一场景确有相对测距动态；不能仅以共同转弯存在为依据。\n"
    report+='''
### 失败过程与解释

- 30 s 内未确认恢复的 20 个事件中，14 个在更长的既有仿真尾段内恢复；另 6 个直到 600 s 仍未恢复。这里未补跑或延长仿真，也未改正式 30 s 终点。seed 30 / event 8 的首次重新准入恰在故障结束后第 30 s，稳定确认需要后续 2 个历元，故仍属于正式期限内未确认恢复，不能改计成功。
- 预定主图使用 seed 1 / event 1，展示异常边 F1–L1 与同 follower 健康边 F1–L2。附加失败图按 seed、事件升序选择首个失败，即 seed 5 / event 2（F2–L2），不是挑选有利实例。此故障 135 s 结束，直到 246 s 才开始稳定准入，延迟 111 s。
- 上述失败实例故障结束后 30 s 内，归一化创新绝对值中位数约 1.123，预测距离相对真值偏差中位数约 −0.870 m；距离水平冻结、报警和隔离占用均为 100%。日志与代码一致地支持：冻结保护下仍保留的预测水平偏差，使正常创新解除门限难以连续满足，可能造成恢复迟滞。冻结期间速率可继续适应，不能解释成整个预测器停止运动。
- 唯一未隔离事件是 seed 40 / event 13，F2–L1：2 s 后阈值越界、3 s 后确认报警，但同一 follower 的另一条 F2–L2 边处于被选择隔离状态。它暴露了“每 follower 最多选一条边”的选择竞争风险，不能以报警记录替代实际隔离证据。
- 健康机动中的误触发也伴随持续创新及水平冻结。当前数据支持“该健康动态场景会发生错误状态切换”，不能单凭本轮一条轨迹把全部误差归因于转弯，也不能据此断言所有健康机动都会失败。

完整事件及原因见 `tables/E3/events_per_seed.csv`、`recovery_failure_diagnostics.csv`、`missed_isolation_diagnostics.csv`、`healthy_isolation_episodes.csv`。这些结果属于本轮有效结论，没有为改善结果改算法或重新选参。

## 4. E2a：规模与几何、故障暴露

保持 3 个 leader（原数据列 11、14、15）；2F/3F/5F 的 follower 源列分别为 [1,2]、[1,2,3]、[1,2,3,4,21]。节点编号采用 F1…FF，随后 L1…L3；跨规模以角色 ID 和源列对应身份。新增列 4、21 在正式结果生成前按通信与几何有效性选择，未按定位效果选点。

'''
    report+=tab(['配置','F 数','名义测距边','leader 几何秩','最大 cond(H)','最大 leader 距离 m','故障/可观测 leader edge-epoch','故障占比'],geomrows)
    report+='''
所有场景在 1–600 s 的每个测距更新上都连通；每 follower 有 3 条名义 leader 边。H 的行是三条单位视线向量，表中为其秩与二维范数条件数（cond₂）。这是名义的局部 leader 几何诊断，并不等于隔离后的全局图完全可观性证明。隔离期间每 follower 的实际新准入 leader 边数最低为 2；其分 seed 统计见 `admitted_constraints_per_seed.csv`。

2F 保留 F1/F2 原事件和对应随机幅值；5F 在保留前三个 follower 的原事件后，为 F4/F5 分别复用 F1/F2 的故障时间角色，新增幅值独立抽取。所有 follower 均受故障影响，但每时刻每 follower 最多一条真实异常 leader 边。完整暴露比例（含各 follower、完整时间与去启动期两种分母）见 `fault_exposure.csv`。

跨规模保持噪声分布与 seed 集合、原角色轨迹及故障映射；由于噪声数组维度和随机数消费顺序改变，不声称不同规模中同名 follower 的噪声逐元素相同。严格输入配对针对每个场景的四方法；跨 NLOS 强度则进一步做到完整输入数组摘要相同。规模间的几何、新增协同边、故障角色构成及噪声 realization 均可能影响趋势，等权 macro 不能消除这些因素。

## 5. E2b：强度定义与配对

沿用当前段级 NLOS 生成器：先按 0.60/0.40 选择均值 3/6 m、标准差 0.6/1.0 m 的分量，再在所选分量内重采样直至落入 [1,8] m。每事件抽一次，段内固定。这里明确保留“先选分量、分量内截断”的真实实现。

对同 seed、同事件的基准偏差 b，弱/基准/强分别为 0.5b、b、1.5b，支持区间对应 [0.5,4]、[1,8]、[1.5,12] m；缩放后不再次截断。三档保持相同轨迹、IMU/位置/测距噪声缓存、事件身份、起止时间与持续时间。正式结果中已核对实际输入摘要和全部事件的幅值比例，未仅凭相同 seed 宣称配对。

## 6. 定位结果与同 seed 配对比较

以下均为 50 个独立 seed 的均值±SD，单位 m。每次完整评价 30001 个时刻；四方法完整完成、有效配对数均为 50。全部 follower 的同类结果在 [全部Follower定位指标.md](全部Follower定位指标.md)，逐 seed 原值在 `tables/E2/metrics_per_seed.csv`。

### Follower 等权 macro-average

'''+position_table('macro')+'''
### Formation pooled overall

'''+position_table('pooled')+'''
### CUSUM-FGO 与其他方法的配对 macro RMSE 差

负值表示 CUSUM-FGO 误差较低；相对降低比例用两方法跨 seed 均值计算，只作描述。SD 来自逐 seed 差值，不是两列 SD 相减。

'''+tab(['场景','配对比较','差值均值 ± SD (m)','相对降低','配对 seed 数'],pairrows)+'\n'
    best=[]
    for s in SCENARIOS:
        m=min(METHOD,key=lambda m:float(lookup[s,m,'macro',0]['RMSE3D_Mean']))
        best.append(f"{SCENE[s]}：{METHOD[m]}")
    report+='各配置中平均 macro RMSE 最低的方法为：'+'；'.join(best)+'。这比较的是跨 seed 均值，不表示每个 follower、每个 seed、每个时刻均最优。\n\n'
    size_values=[float(lookup[s,'cusum_fgo','macro',0]['RMSE3D_Mean']) for s in ('size_2f','baseline_3f','size_5f')]
    strength_values=[float(lookup[s,'cusum_fgo','macro',0]['RMSE3D_Mean']) for s in ('nlos_weak','baseline_3f','nlos_strong')]
    report+=f"CUSUM-FGO 的 2F/3F/5F macro RMSE 均值依次为 {'、'.join(f'{v:.3f}' for v in size_values)} m；弱/基准/强 NLOS 下依次为 {'、'.join(f'{v:.3f}' for v in strength_values)} m。规模趋势须结合第 4 节的约束冗余和角色差异解读。强度趋势不必单调：更强偏差可能更快达到检测/隔离条件，而较弱偏差可能较久保留在优化中；本轮没有专门的机制消融来确证这一因果解释，不能写成噪声越强精度越好的一般规律。\n"
    report+='''
## 7. 图表与可复算证据

图均提供 PDF、SVG（可编辑文字）与 600 dpi PNG；宽 180 mm，正文约 8 pt，图例 7 pt。源表和图注见 [图表与复算索引.md](图表与复算索引.md)。

- `E3_state_chain`：预定 seed 1 / event 1 的异常/健康边对照，以及旧因子随窗口退出。
- `E3_recovery_failure`：首个恢复超时实例 seed 5 / event 2，明确呈现未及时恢复。
- `E3_seed_statistics`：50 个 seed 的条件延迟、恢复失败率及健康隔离占用；所有 seed 都保留。
- `E3_healthy_maneuver`：真实距离变化、50 个 seed 的最大逐边 CUSUM 和健康隔离占用。
- `E2a_formation_size`、`E2b_NLOS_intensity`：四方法 pooled 与 macro RMSE/CDF95 的均值±SD。

`raw/<scenario>/seed_XXXX.mat` 保存完整最终配置、时间、轨迹、噪声缓存、故障段、各方法误差和逐更新日志。`derived/` 是可删除后重建的离线统计及状态矩阵，不是唯一原始数据。日志中的 truth/fault 字段仅供离线分析；实际在线检测器调用没有引入这些输入。

## 8. 验证、版本与环境

- 修改前保存 40 s、seed 1001 的四方法基准；修改后误差、真值、测距、权重及因子准入逐元素完全一致。
- `test_revision_experiments`：幅值唯一变化、实际输入缓存配对、2F/5F 映射与几何、健康相对运动与理想 IMU 导数一致、预测器日志检查通过。
- 现有 `test_stage1_cusum_components`、`test_stage1_four_method_comparison_smoke` 通过；未运行无关的 IMU preintegration 测试。
- `test_revision_analysis.py` 的 10 个可手算统计测试通过，覆盖漏检、预先状态、正常恢复、失败、删失、观察终点处的稳定确认、macro 与历史因子退出。第一次短健康测试对单条边的检查过窄而失败，已改为检查所有 leader 边，原失败日志保留；未改变正式场景定义。
- E3 100/100、E2 250/250（含复用的 50 个基准）检查点完整；唯一场景检查点共 300。输入配对、段内固定幅值、全部 follower 样本、离线 RMSE、完整轨迹几何、实际最多一条 leader 边隔离均核查。正式失败/缺失/中断残留为 0。
- 原始文件 SHA256 与派生统计记录一致，封存的执行代码快照哈希不变。E3/E2 完成验收时活动 MATLAB 代码与快照一致；后续 E1/E4/E5 授权修改另有快照与变更清单，旧结果复算应使用本阶段快照。论文原稿保持原哈希。证据为 `validation/delivery_audit_at_E3_E2_completion.json`、`validation/delivery_audit.json`、`tables/raw_manifest.csv` 和 `provenance/`。

'''
    report+=f"实际环境：MATLAB {audit['versions']['matlab']}；Python {audit['versions']['python'].split()[0]}、NumPy {audit['versions']['numpy']}、SciPy {audit['versions']['scipy']}、Matplotlib 3.11.2，Windows。4 个独立 MATLAB 进程按 seed 分片，各用单计算线程。运行总耗时仅用于操作记录，不构成 E4 的单次在线更新或硬实时性能证据。\n\n"
    report+='''工作区原先已有未提交修改，因此使用任务前快照、实际执行快照、补丁及 SHA256 保存可恢复版本；不能仅用 Git HEAD 代表实际运行代码。保留现有工作分支和用户修改，未自动提交混合工作区，也未推送 GitHub。辅助依赖、脚本、临时/预览文件放在 `D:/codex_agent` 分类目录；正式材料位于本项目。

## 9. 文件变更与复算入口

| 文件（均相对项目） | 本轮关键变化 |
| --- | --- |
| code/run_stage1_cusum_comparison.m | opt-in 编队映射、相对运动、段幅值缩放与完整输入日志 |
| code/stage1_cusum_default_config.m | 新场景开关及 identity 默认值，保留原默认路径 |
| code/compute_ekf_cusum_range_weights.m | 增加真实预测器状态日志，未改状态更新公式 |
| code/revision_experiment_config.m / revision_validate_config.m | 六个冻结配置、有效性保护 |
| code/revision_relative_motion.m | 独立健康相对机动及解析导数 |
| code/run_revision_experiments.m | 分 seed 原始检查点、E3→E2 验收门、失败记录和输入核对 |
| code/test_revision_experiments.m | MATLAB 回归及新场景测试 |
| code/analyze_revision_experiments.py / test_revision_analysis.py | 事件、健康占用、三层定位统计与边界测试 |
| code/diagnose_revision_e3.py | 固定终点之外的既有尾段诊断、误隔离实例 |
| code/audit_revision_delivery.py | 哈希、角色映射、暴露、几何、有效约束和交付审计 |
| code/plot_revision_experiments.py / write_revision_report.py | 图与源数据、自动生成结果文档 |
| code/revision_python_requirements.txt | Python 复算依赖版本 |
| AGENTS.md / 根目录执行提示词 / CHANGELOG.md | 按本轮指令保留本地工作及记录修改来源 |

命令和快照恢复方式见 `README.md` 与 `图表与复算索引.md`。正式结果目录独立创建，没有覆盖原有基准。

## 10. 论文应采用的结论及边界

本轮证据能直接补充不同编队规模、NLOS 强度及异常处理状态过程，并支撑当前测试条件下的定位性能比较。论文应补全全部 follower 和 formation 两种聚合，明确故障模型、事件口径、seed 独立重复及实际算法状态。

应明确承认：同一 follower 同时多条严重异常边未被本轮验证；旧报警持续时，即使真实注入仍为每 follower 一条，也可能发生选择竞争；健康相对机动存在误隔离；故障结束后并不保证 30 s 内稳定恢复。历史窗口退出不等于即时删除。规模、几何、约束冗余与噪声维度变化不能完全分离。当前结论来自规定轨迹、噪声模型与仿真，不能直接扩展为实飞、任意拓扑或机载硬实时保证。

写作材料另存 `论文/修订记录/E3_E2_实验结果与修订建议.md`。本报告仅描述 E3/E2 已验证结果；后续授权实验另有报告，正式稿保持原样。
'''
    (output/'E3_E2_实验结果报告.md').write_text(report,encoding='utf-8')
    follower=['# 全部 follower 定位指标\n','50 个 seed（1–50）的均值±样本 SD；每个 follower 每 seed 30001 个三维误差样本。单位 m。\n']
    for s in SCENARIOS+['healthy_maneuver']:
        rows=[r for r in (position if s!='healthy_maneuver' else healthy_position) if r['Scenario']==s and r['Aggregation']=='follower']
        follower+=['\n## '+SCENE[s]+'\n',tab(['Follower','方法','Maximum','Mean','CDF95','3D RMSE'],
            [[r['Follower'],METHOD[r['Method']]]+[ms(r,key) for key in METRICS] for r in sorted(rows,key=lambda r:(int(r['Follower']),list(METHOD).index(r['Method'])))])]
    (output/'全部Follower定位指标.md').write_text('\n'.join(follower),encoding='utf-8')
    suggestions=ROOT/'论文'/'修订记录'; suggestions.mkdir(parents=True,exist_ok=True)
    text=f'''# E3 / E2 实验结果与论文修订建议

日期：2026-10-03。仅为写作素材，未修改 EIC_English.docx。原始结果、源数据及图表位于项目 `结果/2026-10-03_E3_E2_revision/`。

## 回应 R2.3：隔离、误隔离和恢复

建议新增一个机制验证小节，先区分 threshold crossing、confirmed alarm、actual exclusion、re-admission，再展示预定 seed 1 状态图与全体统计。基准 50 个 seed 共 650 个故障，647 个在故障内新越过阈值、3 个阈值状态预先存在；646 个故障内新隔离、3 个预先隔离、1 个未隔离。条件检测延迟为 {ms(event_summary,'DetectionDelay')} s，隔离延迟为 {ms(event_summary,'IsolationDelay')} s。

在 649 个曾隔离事件中，629 个在固定 30 s 内满足连续 3 个历元的稳定准入，20 个未满足。按 seed 汇总的恢复失败率为 {ms(event_summary,'RecoveryFailureRate',100)}%；成功事件条件恢复延迟为 {ms(event_summary,'RecoveryDelay')} s。不可写“所有异常结束后均及时恢复”。补充失败图与解释：距离水平冻结可保留预测偏差，延缓正常创新解除条件满足；冻结期间速度仍可能更新。

健康压力场景无 NLOS，260–460 s 有明确相对机动。16/50 个 seed 出现误报警和实际误隔离，共 16 个事件；健康边历元误隔离占用 2100/263250=0.798%，4 个事件持续到仿真结束。应同时报告运行发生比例和占用比例，不能只用低占用率宣称不存在机动误触发。把这一点写入 Discussion 的适用边界。

明确说明隔离停止新因子准入，旧因子继续保留到滑窗自然退出。每 follower 最多选择一条主要异常 leader 边，多条真实严重异常同时存在未在本轮验证；持续旧报警也可能与新异常竞争。

## 回应 R2.2：规模和 NLOS 强度

新增两个独立实验：2F3L/3F3L/5F3L 固定基准 NLOS；3F3L 固定轨迹和事件，偏差幅值取 0.5/1/1.5 倍。分别引用 `E2a_formation_size` 和 `E2b_NLOS_intensity`，无需全因子组合。

四方法各配置均使用 50 个 seed。共同基准只运行一批并复用。全部 follower、pooled、等权 macro 的 Maximum/Mean/CDF95/RMSE 已齐备。主表可用 macro 与 pooled 关键指标，完整 follower 表放补充材料。须解释 macro CDF95/Maximum 的特殊含义，并注明误差条是跨 seed SD，不能把时序样本数写成独立 n。

CUSUM-FGO 在 2F/3F/5F 的 macro RMSE 均值分别为 {'、'.join(f'{v:.3f}' for v in size_values)} m；弱/基准/强分别为 {'、'.join(f'{v:.3f}' for v in strength_values)} m。四方法完整数据应以结果报告对应表为准，不概括成“任意情况下均优于其他方法”。

补充配置表：源节点身份、有效测距边 7/12/25 条、500 m 距离阈值、局部 leader 几何秩 3、条件数及故障 edge-epoch 比例。说明等权汇总不能消除规模间几何、角色构成和约束冗余差异；跨规模仅保持噪声模型相同，跨强度才核查了共同输入数组完全配对。

## 图注与方法文字应统一

使用 alarm / confirmed alarm / isolation / re-admission 等词时明确其对应状态。恢复时间取连续 3 次稳定准入的第 1 次，但要求整个确认过程在观察期内。区分未隔离、不适用、完整观察失败及提前删失；本轮正式恢复删失为 0。

每图注明 seed 的选择规则、n=50 的独立重复定义、中心与离散程度、固定参数 h=5/κ=0.5、数据表位置。不能把图优化 1 Hz 的设置本身当作已测得的实时性结论，本轮未执行 E4。

## 后续边界

本阶段没有调参、增加多异常边算法或加入新恢复机制。后续 E1/E4/E5 属于用户另行授权的剩余实验，见 `../2026-10-03_E1_E4_E5_revision/`。正式论文改写与审稿回复可直接引用本结果，但论文原稿尚未修订。用户先前排除的 FGO 对照定义和历史 Table 2 数值核查不在此列。
'''
    (suggestions/'E3_E2_实验结果与修订建议.md').write_text(text,encoding='utf-8')
    print('Created result report, all-follower tables and manuscript revision notes.')


if __name__=='__main__': main()
