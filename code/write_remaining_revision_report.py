"""Generate E1/E4/E5 and overall experiment materials only after validation."""
import csv
import json
from collections import Counter
from pathlib import Path
from analyze_remaining_revision import ROOT, OUTPUT, FIRST
from analyze_revision_experiments import METRICS
from write_revision_report import tab, ms, METHOD, SCENE


def read(stage, name, output=OUTPUT):
    with (output/'tables'/stage/name).open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def num(row, key, digits=3, multiplier=1):
    return f'{float(row[key])*multiplier:.{digits}f}'


def get(rows, **criteria):
    found=[r for r in rows if all(str(r[k])==str(v) for k,v in criteria.items())]
    assert len(found)==1,(criteria,len(found))
    return found[0]


def save(name, text, output=OUTPUT):
    (output/name).write_text(text.strip()+'\n', encoding='utf-8')


def main(output=OUTPUT):
    gates={s:json.loads((output/f'{s}_VALIDATED.json').read_text()) for s in ('E1','E4','E5')}
    assert all(g['passed'] for g in gates.values())
    for s,n in (('E1',900),('E4',30),('E5',150)):
        assert gates[s]['completed_checkpoints']==n and gates[s]['missing_checkpoints']==0
    manifest=json.loads((output/'frozen_manifest.json').read_text(encoding='utf-8-sig'))
    assert manifest['no_parameter_reselection'] and manifest['main_h']==5 and manifest['main_kappa']==.5
    p1=read('E1','position_summary.csv',output); health=read('E1','health_summary.csv',output)
    events=read('E1','event_summary.csv',output); health_seed=read('E1','health_per_seed.csv',output)
    event_rows=read('E1','events_per_seed.csv',output)
    points=[f'h{h}_k{k:g}'.replace('.','p') for h in (3,5,7) for k in (.25,.5,.75)]
    e1rows=[]; erows=[]
    for point in points:
        for scene in ('baseline_3f','healthy_maneuver'):
            pos=get(p1,Point=point,Scenario=scene,Method='cusum_fgo',Aggregation='macro',Follower=0)
            pooled=get(p1,Point=point,Scenario=scene,Method='cusum_fgo',Aggregation='pooled',Follower=0)
            hr=get(health,Point=point,Scenario=scene,Method='cusum_fgo',EdgeScope='leader',Phase='all')
            seeds=[r for r in health_seed if r['Point']==point and r['Scenario']==scene and
                   r['Method']=='cusum_fgo' and r['EdgeScope']=='leader' and r['Phase']=='all']
            assert len(seeds)==50
            alarms=sum(int(r['AlarmNewHealthyEpisodes']) for r in seeds)
            iso=sum(int(r['IsolationNewHealthyEpisodes']) for r in seeds)
            runs=sum(int(r['IsolationNewHealthyEpisodes'])>0 for r in seeds)
            e1rows.append([point,SCENE[scene],ms(pos,'RMSE3D'),ms(pooled,'RMSE3D'),
                ms(hr,'AlarmHealthyOccupancy',100),ms(hr,'IsolationHealthyOccupancy',100),
                f'{alarms} / {iso}',f'{runs}/50'])
        es=get(events,Point=point,Scenario='baseline_3f',Method='cusum_fgo')
        er=[r for r in event_rows if r['Point']==point and r['Method']=='cusum_fgo']
        statuses={key:Counter(r[key+'Status'] for r in er) for key in ('Detection','Isolation','Recovery')}
        detection=statuses['Detection']; isolation=statuses['Isolation']; recovery=statuses['Recovery']
        erows.append([point,f"{detection['detected']}/{detection['preexisting']}/{detection['missed']}",
            ms(es,'DetectionDelay')+f" (n={es['DetectionDelay_N']})",
            f"{isolation['detected']}/{isolation['preexisting']}/{isolation['missed']}",
            ms(es,'IsolationDelay')+f" (n={es['IsolationDelay_N']})",
            f"{recovery['recovered']}/{recovery['failed']}/{recovery['censored']}/{recovery['not_applicable']}",
            ms(es,'RecoveryFailureRate',100)])
    fault_macro=[r for r in p1 if r['Scenario']=='baseline_3f' and r['Aggregation']=='macro']
    fault_events=[r for r in events if r['Scenario']=='baseline_3f']
    rmse_range=[float(r['RMSE3D_Mean']) for r in fault_macro]
    delay_range=[float(r['DetectionDelay_Mean']) for r in fault_events]
    healthy_isolation=[r for r in health if r['Scenario']=='healthy_maneuver' and
                       r['EdgeScope']=='leader' and r['Phase']=='all']
    assert all(float(r['IsolationHealthyOccupancy_Mean'])==0 for r in healthy_isolation if r['Point'].endswith('k0p75'))
    e1=f'''# E1 参数敏感性与鲁棒性结果

九个预先冻结网格点，各在基准 NLOS 和无 NLOS 相对机动场景运行 seed 1–50；主方法仅 CUSUM-FGO。中心 h=5、κ=0.5 的 100 个检查点复用 E3，新增 800 次仿真，全部完成。每个点与同 seed 中心的实际噪声、轨迹和测距输入一致。

本实验只分析原参数附近的折中，**保留论文现有 h=5、κ=0.5，不从正式评估结果选参**。没有 calibration、验证集寻优或理论最优声明；未来若需重新选参须另设独立 calibration seeds。

九点基准 macro RMSE 均值范围为 {min(rmse_range):.3f}–{max(rmse_range):.3f} m，条件检测延迟均值为 {min(delay_range):.3f}–{max(delay_range):.3f} s。h 增大时本次故障响应变慢；κ=0.25 的健康误隔离占用明显高于 κ=0.5，κ=0.75 在本次 50 seed 健康压力场景中未观察到报警 / 隔离占用。这是有限场景的观测，不是一般零误报保证。存在比中心某些均值更低的组合，因此不能声称中心最优，或因九点均能运行就声称参数不敏感。

## 定位性能与健康边误触发

单位 m；占用单位 %；均值±样本 SD 的独立单位为 50 个 seed。新事件列为“报警 / 实际误隔离”，运行数只统计本次健康边新隔离。基准 NLOS 行中“健康”指当前未注入偏差的边，故障结束后的持续状态也计入占用，不能把占用与新误触发等同。

{tab(['h / κ 网格点','场景','macro RMSE','pooled RMSE','健康报警占用 %','健康隔离占用 %','新事件数','有新误隔离的 seed'],e1rows)}
## 故障事件与恢复

每点固定 650 个事件。状态计数依次为“故障内新触发 / 预先存在 / 漏检”；恢复计数为“30 s 内稳定恢复 / 30 s 内失败 / 删失 / 未隔离不适用”。条件延迟仅计算成功的新触发或成功恢复，不将漏检填 0；先按 seed 求条件均值，再跨 seed 汇总，表中 n 为有该条件统计的 seed 数。

{tab(['网格点','检测状态计数','条件检测延迟 s','隔离状态计数','条件隔离延迟 s','恢复计数','seed 恢复失败率 %'],erows)}
不同指标可能偏好不同参数，表中结果用于解释误报、响应速度和定位性能的折中。局部九点与有限场景不构成对任意机动或 NLOS 的鲁棒性保证。

图：`figures/E1_parameter_sensitivity`；完整全部 follower、pooled、macro 四指标见 `tables/E1/position_summary.csv`。所有逐 seed、逐事件及健康阶段统计保留在 `tables/E1/`。网格点报警阈值从实际 cfg 读取，并通过 h=3/5/7 的手算边界测试。
'''
    save('E1_参数敏感性结果.md',e1,output)
    runtime=read('E4','runtime_summary.csv',output); timing=read('E4','timing_seeds_per_seed.csv',output)
    runtime_rows=read('E4','timing_per_seed.csv',output); rt=[]; tails=[]; gn=[]
    scenes=('size_2f','baseline_3f','size_5f')
    for scene in scenes:
        for method in METHOD:
            r=get(runtime,Scenario=scene,Method=method,Phase='steady')
            rows=[x for x in runtime_rows if x['Scenario']==scene and x['Method']==method]
            rt.append([SCENE[scene],METHOD[method],ms(r,'AlgorithmSeconds_Mean',1000),
                ms(r,'EndToEndSeconds_Mean',1000),ms(r,'EndToEndSeconds_P95',1000),
                ms(r,'EndToEndSeconds_P99',1000),ms(r,'EndToEndSeconds_Max',1000),
                f"{max(float(x['EndToEndSeconds']) for x in rows)*1000:.3f}",
                f"{sum(float(x['EndToEndSeconds'])>1 for x in rows)}/{len(rows)}"])
            tails.append([SCENE[scene],METHOD[method],ms(r,'AlgorithmSeconds_P95',1000),
                ms(r,'AlgorithmSeconds_P99',1000),ms(r,'AlgorithmSeconds_Max',1000),
                f"{max(float(x['AlgorithmSeconds']) for x in rows)*1000:.3f}"])
            if method in ('fgo','cusum_fgo'):
                group=[x for x in timing if x['Scenario']==scene and x['Method']==method and x['Phase']=='all']
                gn.append([SCENE[scene],METHOD[method],500 if method=='fgo' else 30,
                    ms(r,'GNMean'),max(int(float(x['GNMax'])) for x in group),
                    sum(int(x['GNCapHits']) for x in group),sum(int(x['GNUnconverged']) for x in group),
                    sum(int(x['Updates']) for x in group),
                    f"{max(float(x['GNFinalStepMax']) for x in group):.3g}"])
    overall_max=max(float(r['EndToEndSeconds']) for r in runtime_rows)
    overruns=sum(float(r['EndToEndSeconds'])>1 for r in runtime_rows)
    unconverged=sum(int(r['GNUnconverged']) for r in timing if r['Phase']=='all' and r['GNUnconverged'])
    feasibility=('所有已测完整在线周期均在 1 s 内完成，在当前测试硬件、配置与 1 Hz 更新频率下具有在线运行可行性。'
                 if overruns==0 else f'有 {overruns} 个在线周期超过 1 s，本轮不能声称所有更新均满足 1 Hz 的单周期预算。')
    e4=f'''# E4 四方法计算、收敛与在线可行性

三个规模各 seed 1–10、四种方法；共 30 个检查点、120 次方法运行。单 MATLAB 进程、单计算线程，其他正式仿真完成后执行；所有方法及规模先以 seed 1003 预热 30 s，预热不纳入统计，同 seed 方法顺序按 (seed−1) mod 4 轮换。记录到的定位误差数组与 E2 对应方法及 seed 逐元素一致，计时未改变导航结果。

硬件：AMD Ryzen 9 7945HX，16 物理核 / 32 逻辑处理器，约 16 GB RAM；Windows 11，MATLAB R2025b（25.2.0.2998904）。这是本机的实测软件计算成本，不包含通信、传感器等待、操作系统实时调度或机载处理器验证。

## 两种计时边界

- Algorithm-specific runtime：本方法的观测修正、CUSUM/权重/准入、EKF 测距更新或图构建与求解、协方差和 Kalman 反馈，排除离线传感器/真值/故障包生成。
- End-to-end online update runtime：本 1 s 周期 50 次 SINS 传播的计算时间之和，加共同时间更新、先验和协同处理直到状态反馈完成的 1 Hz 更新时间。计时在模拟 IMU 包生成后开始；GPS/测距离线生成时间扣除；在诊断日志与文件写出前结束。

两个口径有包含关系，不能只凭图求解耗时判断完整在线周期。仪器计时自身存在少量开销；结果仅表示当前软件与硬件的观测可行性。

计时保持现有四方法的实现：FGO 使用单历元图，CUSUM-FGO 使用 10 帧位置滑窗及 SINS 相对位移因子。因此两者差值包含图规模、建图、协方差和反馈等开销，不能全部归为 CUSUM 检测自身。这里比较完整方法实现的实际成本，不重新定义或改造对照。

## 稳定阶段（10–600 s）耗时

单位 ms，表中均值±SD 是“每 seed 的相应统计量，再跨 10 seed 汇总”。填窗期 1–9 s、全时段 1–600 s 单列在完整 CSV。每组稳定阶段每 seed 591 次更新；最后两列报告全时段真实最坏值和预算超限数，避免均值遮蔽尾部。

{tab(['场景','方法','算法 Mean','完整 Mean','完整 P95','完整 P99','完整 Max 的 seed 汇总','全时段绝对 Max','完整 >1 s 次数'],rt)}
算法口径同样保留尾部：

{tab(['场景','方法','算法 P95','算法 P99','算法 Max 的 seed 汇总','算法全时段绝对 Max'],tails)}
## Gauss–Newton 收敛

EKF / CUSUM-EKF 不适用。稳定阶段平均迭代数采用跨 seed 均值±SD；触顶、未收敛和最终增量最大值是全时段统计。FGO 与 CUSUM-FGO 的现有迭代上限分别为 500、30，停止依据为最终增量无穷范数 <1e−5，不能把达到上限本身当成收敛。

{tab(['场景','方法','上限','稳定平均迭代','实际最大迭代','触顶次数','未收敛次数','在线更新数','最终增量绝对最大'],gn)}
全测试完整周期最大值 {overall_max*1000:.3f} ms，超过 1 s 共 {overruns}/{len(runtime_rows)} 次；图优化未收敛共 {unconverged} 次。{feasibility} 未测试机载硬实时，也不外推到任意规模、窗口和故障情形。

图：`figures/E4_runtime_convergence`。逐更新、逐 seed、三阶段全部统计在 `tables/E4/timing_per_seed.csv`、`timing_seeds_per_seed.csv` 和 `runtime_summary.csv`；失败或缺失检查在 `validation.json`。硬件来源、预热与运行日志在 `provenance/` 和 `validation/`。
'''
    save('E4_计算性能与收敛结果.md',e4,output)
    p5=read('E5','position_summary.csv',output); paired=read('E5','paired_center_summary.csv',output)
    p5seeds=read('E5','paired_center_differences_per_seed.csv',output); arows=[]
    names={'full':'A0 完整方法','no_isolation':'A1 无确认隔离','no_soft_weighting':'A2 无软降权'}
    for point,name in names.items():
        macro=get(p5,Point=point,Scenario='baseline_3f',Method='cusum_fgo',Aggregation='macro',Follower=0)
        pool=get(p5,Point=point,Scenario='baseline_3f',Method='cusum_fgo',Aggregation='pooled',Follower=0)
        diff=get(paired,Point=point,Scenario='baseline_3f',Aggregation='macro',Follower=0)
        rows=[r for r in p5seeds if r['Point']==point and r['Aggregation']=='macro']
        arows.append([name]+[ms(macro,m) for m in METRICS]+[ms(pool,'RMSE3D'),
            ms(diff,'RMSE3DDifference'),sum(float(r['RMSE3DDifference'])>0 for r in rows)])
    e5=f'''# E5 基准场景机制消融

仅基准 3F3L NLOS、seed 1–50、原参数 h=5、κ=0.5。A0 复用 E3；A1、A2 各新增 50 次，共 100 次仿真，未扩大到其他规模或强度。

A1 仅移除确认报警后的新因子隔离，保持预测器冻结、CUSUM、报警状态和原软权重；所有当前观测边准入。A2 仅将准入因子的软权重置为 1，保留检测、冻结、确认报警、选择性隔离与恢复。两变体的实际输入、预测器、CUSUM 及报警状态均与 A0 配对核查，开关真实生效。A1 的“报警”仍存在，但不会因报警拒绝新因子。

均值±样本 SD，n=50 seed；单位 m。差值为“变体−A0”，正值表示误差增加，最后一列是 macro RMSE 更高的 seed 数，不作为显著性检验。

{tab(['变体','macro Maximum','macro Mean','macro CDF95','macro RMSE','pooled RMSE','配对 macro RMSE 差','差值 >0 seed'],arows)}
在本次配置中，两变体各有 {arows[1][-1]}/50 与 {arows[2][-1]}/50 个 seed 的 macro RMSE 高于配对完整方法；无隔离的平均误差增量更大。这里是描述性配对结果，未作显著性检验。

消融比较解释的是当前配置中移除某机制的影响。软降权、隔离和历史窗口因素耦合，不能将两个差值视为可相加、完全独立的因果贡献；也不据此对消融单独调参。

图：`figures/E5_mechanism_ablation`。全部 follower / pooled / macro、配对差值和状态源表均在 `tables/E5/`。
'''
    save('E5_机制消融结果.md',e5,output)
    followers=['# E1 / E4 / E5 全部 follower 与 formation 指标','单位 m。先按 seed 计算各 follower 指标；macro 为 follower 指标等权算术平均，pooled 为合并误差样本后计算。macro CDF95 不是整体分位数，macro Maximum 不是整体最坏值。不同角色、故障暴露和约束冗余的构成差异仍需说明。']
    for stage,rows in (('E1',p1),('E4',read('E4','position_summary.csv',output)),('E5',p5)):
        followers+=['\n## '+stage+'\n',tab(['配置','场景','方法','汇总','Follower','Maximum','Mean','CDF95','RMSE'],
            [[r['Point'],SCENE[r['Scenario']],METHOD[r['Method']],r['Aggregation'],r['Follower']]+
             [ms(r,m) for m in METRICS] for r in rows])]
    save('全部Follower与Formation指标.md','\n\n'.join(followers),output)
    overall=f'''# 论文返修实验整体验收报告

日期：2026-10-03。E3→E2→E1→E4→E5 已按顺序完成并通过各阶段验收。本轮实验整体结束；正式论文 Word、审稿回复和参考文献修订仍未执行。全部在本地完成，未推送 GitHub。主方法保持原 h=5、κ=0.5。

## 完成范围与工作量

| 实验 | 配置与独立重复 | 实际新运行 | 主要材料 |
| --- | --- | --- | --- |
| E3 隔离 / 恢复 | 基准与健康相对机动，各 50 seed | 100 检查点，300 方法运行 | ../2026-10-03_E3_E2_revision/E3_E2_实验结果报告.md |
| E2 规模 / NLOS | 2F/3F/5F；3F 弱/基准/强，各 50 seed，基准复用 | 200 检查点，800 方法运行 | 同上及全部 Follower 表 |
| E1 h–κ 敏感性 | 九点×两场景×50 seed，中心复用 100 | 800 检查点 / 方法运行 | E1_参数敏感性结果.md |
| E4 计算 / 收敛 | 三规模×10 seed×四方法，独立串行计时 | 30 检查点，120 方法运行 | E4_计算性能与收敛结果.md |
| E5 机制消融 | A0/A1/A2 各 50 seed，A0 复用 | 100 检查点 / 方法运行 | E5_机制消融结果.md |

合计 **1230 个新唯一检查点、2120 次正式方法仿真**，不含预热和开发测试。复用不是新独立重复；各阶段检查点计数在 E3/E2、E1、E5 间不能直接相加为独立样本数。

## 可以支持的结论及必要限制

- E2：本次五个配置的 CUSUM-FGO macro RMSE 均值低于三个现有对照；全部 follower、pooled 和等权 macro 已补齐。不同规模的几何、故障角色构成、冗余和随机数组维度仍影响比较，不能全部归因于规模。
- E3：650 个故障中有 1 个未隔离；649 个曾隔离事件有 20 个未在 30 s 内确认稳定恢复。无 NLOS 相对机动场景有 16/50 seed 出现误隔离，4 个直到 600 s 仍未解除。相关失败原样保留，应写入 Discussion；未重新设计恢复算法或多异常边处理。
- E1：九点揭示误报、条件检测延迟与定位表现的折中，不进行评估集寻优；中心参数保留原值，健康压力场景与 E3 相同。
- E4：全测试完整周期最大 {overall_max*1000:.3f} ms，>1 s 为 {overruns}/{len(runtime_rows)} 次，图优化未收敛 {unconverged} 次。{feasibility} 收敛、尾部及预算结果都须与均值共同报告。
- E5：仅做两项基准消融，macro RMSE 为完整 1.065 m、无隔离 1.484 m、无软权重 1.104 m；两个变体均有 50/50 seed 高于对应完整方法。完整表与配对差值保留，不把耦合机制解释为可独立相加的贡献。

本轮证据仅覆盖既定仿真、噪声、几何、窗口及主要单异常 leader 边情形。图中离散程度是跨 seed SD；同一时间序列的历元不作为独立实验样本。没有新增无关参数扫描或全因子实验。

## 验证与可复算材料

MATLAB 新实验 / 四方法回归测试通过；关闭计时和 full 消融保持基准逐元素一致，计时开关也保持导航结果一致。统计器手算测试通过，实际输入配对、E4 数值结果、E5 状态和开关语义逐 seed 核查。正式失败、缺失和 partial 检查点均为 0。图提供 PDF / SVG / 600 dpi PNG；源代码审查、实际导出检查及最终视觉检查记录在 validation/。

执行时 MATLAB 与初始数据快照、输入与原始文件摘要、最终 cfg、种子、硬件与日志均保存。E3/E2 与 E1/E4/E5 分别使用各自执行快照复算；当前 Git HEAD 不代表任务开始时已经存在的未提交代码。原正式论文哈希不变，旧结果未覆盖。详见 README.md、图表与复算索引.md 和 validation/delivery_audit.json。

## 论文写作的后续材料

实验写作建议在 `论文/修订记录/E1_E4_E5_实验结果与修订建议.md`，早期 E3/E2 建议另存。贡献与最相关工作直接对比、术语、方法复现说明、图示、语言和 Reference 2 元数据核实仍属论文修订步骤；本轮实验完成不等于论文返修全文已完成。用户已排除的 FGO 对照定义与历史 Table 2 数据核查未重新开启。
'''
    save('论文返修实验整体验收报告.md',overall,output)
    relative=output.relative_to(ROOT).as_posix()
    first_relative=FIRST.relative_to(ROOT).as_posix()
    (ROOT/'实验结果总索引.md').write_text(f'''# 论文返修实验结果总索引

2026-10-03，E3→E2→E1→E4→E5 全部完成并通过实验验收。共 1230 个新唯一检查点、2120 次正式方法运行，失败与缺失 0；中心 / A0 复用不另计独立重复。主参数 h=5、κ=0.5 保持，未推送 GitHub，正式 Word 尚未修改。

- [整体验收报告]({relative}/论文返修实验整体验收报告.md)
- [E3/E2 结果与风险]({first_relative}/E3_E2_实验结果报告.md)
- [E1 参数敏感性]({relative}/E1_参数敏感性结果.md)
- [E4 两种耗时与收敛]({relative}/E4_计算性能与收敛结果.md)
- [E5 基准机制消融]({relative}/E5_机制消融结果.md)
- [全部 follower / formation 表]({relative}/全部Follower与Formation指标.md)
- [图表与源数据]({relative}/图表与复算索引.md)
- [复算、配置、种子与快照说明]({relative}/README.md)
- [论文写作素材](论文/修订记录/E1_E4_E5_实验结果与修订建议.md)

必须保留的结论边界：E3 有健康误隔离和恢复超时；E1 不作评估集寻优；E4 只支持当前硬件与 1 Hz 的计算可行性，完整周期最大 {overall_max*1000:.3f} ms，未验证机载硬实时。E5 只覆盖基准场景。正式论文、参考文献核查与逐条回复仍属后续论文修订工作。
''',encoding='utf-8')
    notes=ROOT/'论文'/'修订记录'; notes.mkdir(parents=True,exist_ok=True)
    (notes/'E1_E4_E5_实验结果与修订建议.md').write_text(
        '# E1 / E4 / E5 写作素材\n\n未修改正式 Word。以下为已验证实验的配置、结果及边界，完整源表和图在项目结果目录。\n\n'+e1+'\n\n'+e4+'\n\n'+e5,encoding='utf-8')
    print('Created validated E1/E4/E5 reports, all-follower tables, overall report and writing materials.')


if __name__=='__main__': main()
