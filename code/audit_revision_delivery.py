"""Final offline audit of frozen E3/E2 results, traceability and exposure."""
import hashlib
import json
import platform
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

from analyze_revision_experiments import DEFAULT_OUTPUT, ROOT, E2_SCENARIOS, write_csv, loadmat, records
import numpy as np
import scipy


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(8*1024*1024),b''): h.update(chunk)
    return h.hexdigest()


def audit_figures(output=DEFAULT_OUTPUT):
    """Check actual exports, supplementing the plotting-source preflight."""
    from PIL import Image
    rows=[]
    for name in ('E3_state_chain','E3_recovery_failure','E3_seed_statistics',
                 'E3_healthy_maneuver','E2a_formation_size','E2b_NLOS_intensity'):
        base=output/'figures'/name
        with Image.open(base.with_suffix('.png')) as im:
            width,height=im.size; dpi=im.info['dpi']
        svg=ET.parse(base.with_suffix('.svg')).getroot()
        text_count=len(list(svg.iter('{http://www.w3.org/2000/svg}text')))
        width_mm=float(svg.attrib['width'].removesuffix('pt'))*25.4/72
        height_mm=float(svg.attrib['height'].removesuffix('pt'))*25.4/72
        selectable=sum(len(node.text or '') for node in svg.iter('{http://www.w3.org/2000/svg}text'))
        assert text_count>10
        assert abs(width_mm-180)<.1 and height_mm<=175
        assert min(dpi)>599 and abs(width/min(dpi)*25.4-180)<.1 and selectable>100
        rows.append(dict(Figure=name,PNGWidth=width,PNGHeight=height,DPI=dpi,
            SVGWidthMM=width_mm,SVGHeightMM=height_mm,SVGTextElements=text_count,
            SVGTextCharacters=selectable))
    (output/'validation'/'figure_exports.json').write_text(
        json.dumps(dict(passed=True,figures=rows),indent=2),encoding='utf-8')
    print('Actual SVG/PNG export checks passed for all six figures.')


def audit(output=DEFAULT_OUTPUT):
    findings=[]; manifests=[]; geometries=[]; exposure=[]; dynamics=[]; constraints=[]
    scenarios=(*E2_SCENARIOS,'healthy_maneuver')
    all_stats={}
    for scenario in scenarios:
        stats=[]
        for seed in range(1,51):
            raw=output/'raw'/scenario/f'seed_{seed:04d}.mat'
            derived=output/'derived'/scenario/f'seed_{seed:04d}_statistics.json'
            assert raw.exists() and derived.exists(),f'Incomplete {scenario} {seed}'
            cached=json.loads(derived.read_text(encoding='utf-8'))
            digest=sha(raw)
            assert digest==cached['raw_sha256'],f'Raw file changed after analysis: {raw}'
            assert cached['analyzer_sha256']==sha(ROOT/'code'/'analyze_revision_experiments.py')
            manifests.append(dict(Scenario=scenario,Seed=seed,Path=str(raw.relative_to(output)),
                                  Bytes=raw.stat().st_size,SHA256=digest))
            stats.append(cached['data'])
            tr=dict(np.load(output/'derived'/scenario/f'seed_{seed:04d}_cusum_fgo_trace.npz'))
            other=dict(np.load(output/'derived'/scenario/f'seed_{seed:04d}_cusum_ekf_trace.npz'))
            # Detector and admission equality do not imply equal navigation results.
            for key in ('z','cplus','cminus','weight','alarm','admitted','frozen'):
                assert np.array_equal(tr[key],other[key],equal_nan=True),(scenario,seed,key)
            assert np.all(tr['observed']),f'Unexpected natural missing edge {scenario} {seed}'
            assert not np.any(tr['frozen']&tr['level_corrected'])
            F=int(tr['followers']); leader=tr['pairs'][:,1]>F; time=tr['time']; eligible=time>=16
            for f in range(1,F+1):
                edge=(tr['pairs'][:,0]==f)&leader
                active=np.sum(tr['admitted'][:,edge],axis=1)
                assert np.min(active)>=2,f'More than one leader edge isolated: {scenario} {seed} F{f}'
                assert np.max(np.sum(tr['fault'][:,edge],axis=1))<=1
                constraints.append(dict(Scenario=scenario,Seed=seed,Follower=f,
                    MinAdmittedLeaderEdges=int(active[eligible].min()),
                    MeanAdmittedLeaderEdges=float(active[eligible].mean()),
                    TwoLeaderEpochs=int(np.sum(active[eligible]==2)),EvaluationEpochs=int(eligible.sum())))
            if seed==1:
                for f in range(1,F+1):
                    edge=(tr['pairs'][:,0]==f)&leader
                    for label,mask in (('full',np.ones(len(time),bool)),('post_startup',eligible)):
                        fault=int(tr['fault'][mask][:,edge].sum()); total=int(tr['observed'][mask][:,edge].sum())
                        exposure.append(dict(Scenario=scenario,Follower=f,Window=label,
                            FaultEdgeEpochs=fault,ObservedLeaderEdgeEpochs=total,FaultFraction=fault/total,
                            Affected=bool(fault)))
                for j,pair in enumerate(tr['pairs']):
                    if not leader[j]: continue
                    dynamics.append(dict(Scenario=scenario,Follower=int(pair[0]),Leader=int(pair[1]-F),
                        MinRange=float(tr['true_range'][:,j].min()),MaxRange=float(tr['true_range'][:,j].max()),
                        RangeExcursion=float(np.ptp(tr['true_range'][:,j])),
                        MaxAbsRangeRate=float(np.max(np.abs(tr['range_rate'][:,j]))),
                        MaxAbsRangeAcceleration=float(np.max(np.abs(tr['range_acceleration'][:,j])))))
        all_stats[scenario]=stats
        assert len({r['signatures']['truth_xyz'] for r in stats})==1,'Seed-dependent nominal trajectory'
        assert len({r['signatures']['leader_truth_xyz'] for r in stats})==1
        geometries.extend(stats[0]['geometry'])
        payload=loadmat(output/'raw'/scenario/'seed_0001.mat',simplify_cells=True)['payload']
        cfg=payload['cfg']
        fields=['uav_num','high_num','dt','t_stop','communication_range','sigma_dis','sliding_window_length',
            'ekf_cusum_lambda','ekf_cusum_kappa','ekf_cusum_alarm_on_threshold','ekf_cusum_alarm_off_threshold',
            'ekf_cusum_alarm_confirm_epochs','ekf_cusum_alarm_release_epochs','ekf_cusum_calibration_samples',
            'ekf_cusum_innovation_gate','ekf_cusum_release_innovation_gate','ekf_cusum_weight_deadzone',
            'ekf_cusum_weight_gain','ekf_cusum_weight_min','ekf_cusum_range_predictor_alpha',
            'ekf_cusum_range_predictor_beta','ekf_cusum_range_predictor_std','ekf_cusum_predictor_freeze_threshold',
            'ekf_cusum_frozen_rate_adapt_gain','ekf_cusum_frozen_rate_gate','fgo_cusum_detector_mode',
            'fgo_cusum_soft_weight_mode','sliding_window_alarm_exclusion_mode','imu_preintegration_enable',
            'range_predictor_opposite_step_reset_enable','fgo_cusum_opposite_step_reset_enable',
            'revision_nlos_amplitude_scale','revision_follower_source_indices','base_leader_source_indices']
        def plain(v): return v.tolist() if isinstance(v,np.ndarray) else v.item() if isinstance(v,np.generic) else v
        (output/'provenance'/f'effective_config_{scenario}.json').write_text(
            json.dumps({key:plain(cfg[key]) for key in fields},ensure_ascii=False,indent=2),encoding='utf-8')
    # Nested roles share their pre-drawn segment amplitudes even though dimensions change.
    for seed in range(1,51):
        base={e['DrawIndex']:e for e in all_stats['baseline_3f'][seed-1]['schedules']}
        for scenario in ('size_2f','size_5f'):
            for e in all_stats[scenario][seed-1]['schedules']:
                if e['DrawIndex'] in base:
                    ref=base[e['DrawIndex']]
                    assert all(ref[k]==e[k] for k in ('Start','End','Follower','Leader','Bias'))
    write_csv(output/'tables'/'raw_manifest.csv',manifests)
    write_csv(output/'tables'/'geometry_all_scenarios.csv',geometries)
    write_csv(output/'tables'/'fault_exposure.csv',exposure)
    write_csv(output/'tables'/'range_dynamics.csv',dynamics)
    write_csv(output/'tables'/'admitted_constraints_per_seed.csv',constraints)
    # Immutable execution snapshot and manuscripts; allowed changed sources are explicit.
    executed=json.loads((output/'provenance'/'executed_source_manifest.json').read_text(encoding='utf-8-sig'))
    later_source_changes=[]
    for row in executed:
        assert sha(output/'provenance'/'executed_source'/row['path'])==row['sha256'], (
            f'Execution snapshot changed: {row["path"]}')
        if sha(ROOT/row['path'])!=row['sha256']: later_source_changes.append(row['path'])
    allowed={'code/compute_ekf_cusum_range_weights.m','code/run_stage1_cusum_comparison.m',
        'code/stage1_cusum_default_config.m','AGENTS.md','E3_E2_论文返修实验_Codex执行提示词.md','CHANGELOG.md'}
    before=json.loads((output/'provenance'/'before_manifest.json').read_text(encoding='utf-8-sig'))
    preserved=[]
    for row in before:
        if row['path'] not in allowed:
            # Later authorized work may change the active project. Validate
            # the immutable before snapshot; manuscripts are checked live.
            saved=output/'provenance'/'before'/row['path']
            target=saved if saved.exists() else ROOT/row['path']
            assert sha(target)==row['sha256'],f'Preserved file changed: {row["path"]}'
            if row['path']=='论文/EIC_English.docx': assert sha(ROOT/row['path'])==row['sha256']
            preserved.append(row['path'])
    failures=list((output/'raw').glob('*/*_failure_*.mat'))
    partial=list((output/'raw').glob('*/*.partial.mat'))
    if failures or partial: findings.append('Raw failure or incomplete checkpoint requires review')
    versions=dict(python=sys.version,numpy=np.__version__,scipy=scipy.__version__,platform=platform.platform(),
                  matlab=payload['matlab_version'])
    validation=dict(passed=not findings,unique_scenario_seed_runs=len(manifests),
        individual_method_runs=sum(2 if r['Scenario']=='healthy_maneuver' else 4 for r in manifests),
        failed_attempts=len(failures),partial_checkpoints=len(partial),findings=findings,
        raw_bytes=sum(r['Bytes'] for r in manifests),preserved_files=preserved,
        executed_source_hashes_unchanged=True,raw_hashes_match_analysis=True,
        active_sources_match_execution_snapshot=not later_source_changes,
        later_authorized_source_changes=later_source_changes,
        trajectories_seed_invariant=True,nested_fault_amplitudes_match=True,
        cusum_detector_admission_traces_identical_between_methods=True,
        versions=versions)
    (output/'validation'/'delivery_audit.json').write_text(json.dumps(validation,ensure_ascii=False,indent=2),encoding='utf-8')
    assert validation['passed'],validation
    print(json.dumps(validation,ensure_ascii=False,indent=2))


if __name__=='__main__': audit()
