"""Offline E1/E4/E5 analysis with paired-input and mechanism checks."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from analyze_revision_experiments import (ROOT, loadmat, records, digest_array, position_metrics,
    build_trace, trace_statistics, event_seed_summary, grouped_summary, write_csv, METRICS)
import numpy as np

OUTPUT=ROOT/'结果'/'2026-10-03_E1_E4_E5_revision'
FIRST=ROOT/'结果'/'2026-10-03_E3_E2_revision'


def threshold_flags(cplus,cminus,h):
    return np.maximum(cplus,cminus)>=float(h)


def expected_jobs(stage):
    if stage=='E1':
        for h in (3,5,7):
            for k in (.25,.5,.75):
                point=f'h{h}_k{k:g}'.replace('.','p')
                for scenario in ('baseline_3f','healthy_maneuver'):
                    for seed in range(1,51): yield point,scenario,seed
    elif stage=='E4':
        for scenario in ('baseline_3f','size_2f','size_5f'):
            for seed in range(1,11): yield 'runtime',scenario,seed
    elif stage=='E5':
        for point in ('full','no_isolation','no_soft_weighting'):
            for seed in range(1,51): yield point,'baseline_3f',seed
    else: raise ValueError(stage)


def resolve(output,stage,point,scenario,seed):
    folder=output/'raw'/stage/point/scenario
    raw=folder/f'seed_{seed:04d}.mat'
    reuse=folder/f'seed_{seed:04d}.reuse.json'
    if raw.exists(): return raw,False
    if reuse.exists():
        record=json.loads(reuse.read_text(encoding='utf-8-sig'))
        assert record['stage']==stage and record['point']==point and record['seed']==seed
        return ROOT/record['source_relative_to_project'],True
    return None,False


def runtime_rows(payload,method,prefix):
    history=records(payload['methods'][method]['history'])
    graph=method in ('fgo','cusum_fgo'); rows=[]
    for row in history:
        algorithm=float(row['algorithm_specific_seconds']); total=float(row['end_to_end_online_seconds'])
        sins=float(row['sins_interval_seconds']); keyframe=float(row['online_keyframe_seconds'])
        assert min(algorithm,total,sins,keyframe)>0 and int(row['sins_interval_steps'])==50
        assert keyframe>=algorithm and abs(total-sins-keyframe)<1e-12
        rows.append(dict(prefix,Time=float(row['time']),AlgorithmSeconds=algorithm,
            EndToEndSeconds=total,SINSSeconds=sins,KeyframeSeconds=keyframe,
            GraphSolveSeconds=float(row['graph_solve_seconds']),
            GNIterations=float(row['gn_iteration_count']) if graph else math.nan,
            GNFinalStep=float(row['gn_final_step_norm']) if graph else math.nan,
            GNConverged=bool(row['gn_converged']) if graph else None))
    summaries=[]
    for phase,mask in (('all',np.ones(len(rows),bool)),('filling',np.arange(len(rows))<9),('steady',np.arange(len(rows))>=9)):
        group=[r for r,m in zip(rows,mask) if m]; summary=dict(prefix,Phase=phase,Updates=len(group))
        for key in ('AlgorithmSeconds','EndToEndSeconds','SINSSeconds','KeyframeSeconds','GraphSolveSeconds'):
            x=np.array([r[key] for r in group])
            for label,value in (('Mean',x.mean()),('Median',np.median(x)),
                ('P95',np.quantile(x,.95,method='hazen')),('P99',np.quantile(x,.99,method='hazen')),('Max',x.max())):
                summary[key+'_'+label]=float(value)
            summary[key+'_Over1s']=int(np.sum(x>1)); summary[key+'_Over1sRate']=float(np.mean(x>1))
        if graph:
            iterations=np.array([r['GNIterations'] for r in group]); cap=500 if method=='fgo' else 30
            summary.update(GNMean=float(iterations.mean()),GNMax=float(iterations.max()),
                GNCapHits=int(np.sum(iterations>=cap)),GNCapHitRate=float(np.mean(iterations>=cap)),
                GNUnconverged=sum(not r['GNConverged'] for r in group),
                GNUnconvergedRate=float(np.mean([not r['GNConverged'] for r in group])),
                GNFinalStepMean=float(np.mean([r['GNFinalStep'] for r in group])),
                GNFinalStepMax=float(np.max([r['GNFinalStep'] for r in group])))
            assert all(not r['GNConverged'] or r['GNFinalStep']<1e-5 for r in group)
        else:
            summary.update({k:math.nan for k in ('GNMean','GNMax','GNCapHits','GNCapHitRate',
                'GNUnconverged','GNUnconvergedRate','GNFinalStepMean','GNFinalStepMax')})
        summaries.append(summary)
    return rows,summaries


def analyze_one(path,reused,stage,point,scenario,seed,output):
    p=loadmat(path,simplify_cells=True)['payload']; assert p['complete']
    cfg=p['cfg']; h=float(cfg['ekf_cusum_alarm_on_threshold']); k=float(cfg['ekf_cusum_kappa'])
    if stage!='E1': assert (h,k)==(5,.5)
    wanted=('ekf','fgo','cusum_ekf','cusum_fgo') if stage=='E4' else ('cusum_fgo',)
    old_stats=json.loads((FIRST/'derived'/scenario/f'seed_{seed:04d}_statistics.json').read_text(encoding='utf-8'))['data']
    signatures={key:digest_array(p['input_cache'][key]) for key in ('imu_gyro_b','imu_gyro_r','imu_gyro_wg',
        'imu_acc_r','gps_low_standard','gps_high_standard','range_standard')}
    signatures['truth_xyz']=digest_array(p['truth_xyz']); signatures['leader_truth_xyz']=digest_array(p['leader_truth_xyz'])
    assert signatures==old_stats['signatures'],f'Actual input mismatch {stage} {point} {scenario} {seed}'
    if stage=='E4':
        old_payload=loadmat(FIRST/'raw'/scenario/f'seed_{seed:04d}.mat',simplify_cells=True)['payload']
    else: old_payload=None
    old_trace=dict(np.load(FIRST/'derived'/scenario/f'seed_{seed:04d}_cusum_fgo_trace.npz'))
    metrics=[]; events=[]; health=[]; timing=[]; timing_seeds=[]; traces={}
    reference=p['methods'][wanted[0]]
    prefix=dict(Stage=stage,Point=point,Scenario=scenario,Seed=seed,h=h,kappa=k)
    for method in wanted:
        result=p['methods'][method]; mp=dict(prefix,Method=method)
        assert np.array_equal(result['range_error'],reference['range_error'],equal_nan=True)
        for a,b in zip(records(result['history']),records(reference['history'])):
            assert np.array_equal(a['measured_range'],b['measured_range'],equal_nan=True)
            assert np.array_equal(a['graph_prior_positions'],b['graph_prior_positions'])
        for row in position_metrics(result['error_xyz']): metrics.append(dict(mp,**row))
        if stage=='E4':
            assert np.array_equal(result['error_xyz'],old_payload['methods'][method]['error_xyz']),(
                'Runtime instrumentation changed formal result',scenario,seed,method)
            rows,summary=runtime_rows(p,method,mp); timing.extend(rows); timing_seeds.extend(summary)
        if method.startswith('cusum'):
            trace=build_trace(p,method)
            trace['threshold']=threshold_flags(trace['cplus'],trace['cminus'],h)
            assert np.array_equal(trace['measurement'],old_trace['measurement'],equal_nan=True)
            if stage in ('E4','E5'):
                for key in ('z','cplus','cminus','alarm','frozen','prediction','rate_after'):
                    assert np.array_equal(trace[key],old_trace[key],equal_nan=True),(stage,point,key)
                if stage=='E5' and point=='no_isolation':
                    assert np.all(trace['admitted']) and np.array_equal(trace['weight'],old_trace['weight'])
                elif stage=='E5' and point=='no_soft_weighting':
                    assert np.all(trace['weight']==1) and np.array_equal(trace['admitted'],old_trace['admitted'])
                else: assert np.array_equal(trace['admitted'],old_trace['admitted'])
            er,hr=trace_statistics(p,method,trace)
            events.extend(dict(r,Stage=stage,Point=point,h=h,kappa=k) for r in er)
            health.extend(dict(r,Stage=stage,Point=point,h=h,kappa=k) for r in hr)
            dest=output/'derived'/stage/point/scenario/f'seed_{seed:04d}_{method}_trace.npz'
            dest.parent.mkdir(parents=True,exist_ok=True); np.savez_compressed(dest,**trace)
    return dict(metrics=metrics,events=events,health=health,timing=timing,timing_seeds=timing_seeds,
        source=str(path.relative_to(ROOT)),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        reused=reused,stage=stage,point=point,scenario=scenario,seed=seed,input_signatures_match=True)


def run(stage,output=OUTPUT,partial=False):
    fingerprint=hashlib.sha256(Path(__file__).read_bytes()+
        (ROOT/'code'/'analyze_revision_experiments.py').read_bytes()).hexdigest()
    results=[]; missing=[]
    jobs=list(expected_jobs(stage))
    for point,scenario,seed in jobs:
        path,reused=resolve(output,stage,point,scenario,seed)
        if path is None:
            missing.append(dict(Stage=stage,Point=point,Scenario=scenario,Seed=seed)); continue
        dest=output/'derived'/stage/point/scenario/f'seed_{seed:04d}_statistics.json'
        if dest.exists():
            record=json.loads(dest.read_text(encoding='utf-8'))
            if record.get('analyzer_sha256')==fingerprint and record.get('raw_size')==path.stat().st_size:
                results.append(record['data']); continue
        value=analyze_one(path,reused,stage,point,scenario,seed,output)
        dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_text(json.dumps(dict(analyzer_sha256=fingerprint,raw_size=path.stat().st_size,data=value),
            ensure_ascii=False),encoding='utf-8')
        results.append(value); print(f'ANALYZED {stage} {point} {scenario} seed={seed}',flush=True)
    tables=output/'tables'/stage
    merged={key:[row for r in results for row in r[key]] for key in
        ('metrics','events','health','timing','timing_seeds')}
    for key,rows in merged.items(): write_csv(tables/f'{key}_per_seed.csv',rows)
    summary=grouped_summary(merged['metrics'],['Point','Scenario','Method','Aggregation','Follower'],METRICS)
    write_csv(tables/'position_summary.csv',summary)
    health_fields=['HealthyEdgeEpochs','AlarmNewHealthyEpisodes','AlarmOccupiedHealthyEpochs',
        'AlarmHealthyOccupancy','AlarmEpisodesPerEdgeHour','IsolationNewHealthyEpisodes',
        'IsolationOccupiedHealthyEpochs','IsolationHealthyOccupancy','IsolationEpisodesPerEdgeHour']
    write_csv(tables/'health_summary.csv',grouped_summary(merged['health'],
        ['Point','Scenario','Method','EdgeScope','Phase'],health_fields))
    event_seeds=[]
    for point in sorted({r['Point'] for r in merged['health']}):
        hr=[r for r in merged['health'] if r['Point']==point]
        er=[r for r in merged['events'] if r['Point']==point]
        for row in event_seed_summary(er,hr):
            event_seeds.append(dict(row,Stage=stage,Point=point,h=hr[0]['h'],kappa=hr[0]['kappa']))
    write_csv(tables/'event_seed_summary.csv',event_seeds)
    if event_seeds:
        values=[k for k in event_seeds[0] if k not in ('Stage','Point','h','kappa','Scenario','Seed','Method')]
        write_csv(tables/'event_summary.csv',grouped_summary(event_seeds,['Point','Scenario','Method'],values))
    if merged['timing_seeds']:
        values=[k for k in merged['timing_seeds'][0] if k not in
                ('Stage','Point','Scenario','Seed','h','kappa','Method','Phase')]
        write_csv(tables/'runtime_summary.csv',grouped_summary(merged['timing_seeds'],
            ['Scenario','Method','Phase'],values))
    paired=[]
    for row in merged['metrics']:
        if row['Method']!='cusum_fgo': continue
        source=json.loads((FIRST/'derived'/row['Scenario']/f"seed_{row['Seed']:04d}_statistics.json").read_text(encoding='utf-8'))['data']
        old=next(r for r in source['metrics'] if r['Method']=='cusum_fgo' and
            r['Aggregation']==row['Aggregation'] and r['Follower']==row['Follower'])
        paired.append(dict({key:row[key] for key in ('Point','Scenario','Seed','Aggregation','Follower')},
            **{metric+'Difference':row[metric]-old[metric] for metric in METRICS}))
    write_csv(tables/'paired_center_differences_per_seed.csv',paired)
    write_csv(tables/'paired_center_summary.csv',grouped_summary(paired,
        ['Point','Scenario','Aggregation','Follower'],[m+'Difference' for m in METRICS]))
    manifest=[dict(Stage=stage,Point=r['point'],Scenario=r['scenario'],Seed=r['seed'],
        Reused=r['reused'],Source=r['source'],SHA256=r['source_sha256']) for r in results]
    write_csv(tables/'source_manifest.csv',manifest); write_csv(tables/'missing_runs.csv',missing)
    failures=list((output/'raw'/stage).glob('**/*_failure_*.mat'))
    unfinished=list((output/'raw'/stage).glob('**/*.partial.mat'))
    validation=dict(stage=stage,expected_checkpoints=len(jobs),completed_checkpoints=len(results),
        reused_checkpoints=sum(r['reused'] for r in results),new_checkpoints=sum(not r['reused'] for r in results),
        missing_checkpoints=len(missing),failed_attempts=len(failures),partial_files=len(unfinished),
        passed=not missing and not failures and not unfinished,input_pairing_verified=True,
        statistics_unit='seed',parameters_reselected=False,findings=[])
    (tables/'validation.json').write_text(json.dumps(validation,indent=2),encoding='utf-8')
    if not partial:
        assert validation['passed'],validation
        (output/f'{stage}_VALIDATED.json').write_text(json.dumps(validation,indent=2),encoding='utf-8')
    print(json.dumps(validation,indent=2),flush=True)
    return results


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',choices=['E1','E4','E5'],required=True)
    parser.add_argument('--output',type=Path,default=OUTPUT)
    parser.add_argument('--allow-partial',action='store_true')
    args=parser.parse_args(); run(args.stage,args.output,args.allow_partial)
