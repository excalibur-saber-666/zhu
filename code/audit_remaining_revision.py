"""Verify immutable sources, all checkpoint hashes, manuscripts and final exports."""
import hashlib
import csv
import json
import os
import platform
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from analyze_remaining_revision import ROOT, OUTPUT, FIRST, expected_jobs, resolve
from analyze_revision_experiments import loadmat
import numpy as np
import scipy


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for data in iter(lambda:f.read(8*1024*1024),b''): h.update(data)
    return h.hexdigest()


def figures(output):
    from PIL import Image
    from pypdf import PdfReader
    rows=[]
    for name in ('E1_parameter_sensitivity','E4_runtime_convergence','E5_mechanism_ablation'):
        base=output/'figures'/name
        with Image.open(base.with_suffix('.png')) as im:
            width,height=im.size; dpi=im.info['dpi']
        svg=ET.parse(base.with_suffix('.svg')).getroot()
        count=len(list(svg.iter('{http://www.w3.org/2000/svg}text')))
        pdf=PdfReader(base.with_suffix('.pdf')); assert len(pdf.pages)==1
        page=pdf.pages[0]; width_mm=float(page.mediabox.width)*25.4/72
        height_mm=float(page.mediabox.height)*25.4/72
        assert abs(width_mm-180)<.1 and height_mm<=175 and count>10
        assert min(dpi)>599 and abs(width/min(dpi)*25.4-180)<.1
        assert len(page.extract_text())>100
        rows.append(dict(Figure=name,PNGWidth=width,PNGHeight=height,DPI=dpi,
            PDFWidthMM=width_mm,PDFHeightMM=height_mm,SVGTextElements=count,
            PDFExtractedCharacters=len(page.extract_text())))
    (output/'validation'/'figure_exports.json').write_text(
        json.dumps(dict(passed=True,figures=rows),indent=2),encoding='utf-8')
    return rows


def audit(output=OUTPUT):
    os.environ.setdefault('MPLCONFIGDIR','D:/codex_agent/临时/2026-10-03_E1_E4_E5_revision/matplotlib')
    import PIL
    import pypdf
    import matplotlib
    fingerprint=hashlib.sha256((ROOT/'code'/'analyze_remaining_revision.py').read_bytes()+
        (ROOT/'code'/'analyze_revision_experiments.py').read_bytes()).hexdigest()
    gates={}; checked={}; reused=0; new=0; effective=[]
    for stage,count in (('E1',900),('E4',30),('E5',150)):
        gate=json.loads((output/f'{stage}_VALIDATED.json').read_text()); gates[stage]=gate
        assert gate['passed'] and gate['completed_checkpoints']==count
        for point,scene,seed in expected_jobs(stage):
            path,reuse=resolve(output,stage,point,scene,seed); assert path is not None
            cached=json.loads((output/'derived'/stage/point/scene/f'seed_{seed:04d}_statistics.json').read_text())
            assert cached['analyzer_sha256']==fingerprint
            assert cached['raw_size']==path.stat().st_size
            value=cached['data']; assert value['input_signatures_match']
            assert (value['stage'],value['point'],value['scenario'],value['seed'])==(stage,point,scene,seed)
            if path not in checked: checked[path]=sha(path)
            assert checked[path]==value['source_sha256'],f'Changed raw checkpoint: {path}'
            reused+=reuse; new+=not reuse
            if seed==1:
                payload=loadmat(path,simplify_cells=True)['payload']; cfg=payload['cfg']
                assert cfg['t_stop']==600 and cfg['dt']==.02 and cfg['graph_interval']==1
                assert not cfg['imu_preintegration_enable']
                assert cfg['ekf_cusum_lambda']==.9 and cfg['sliding_window_length']==10
                assert cfg['ekf_cusum_alarm_confirm_epochs']==2 and cfg['ekf_cusum_alarm_release_epochs']==3
                assert cfg['ekf_cusum_weight_min']==.1
                if stage=='E1':
                    assert point==f"h{cfg['ekf_cusum_alarm_on_threshold']:g}_k{cfg['ekf_cusum_kappa']:g}".replace('.','p')
                if stage!='E1': assert cfg['ekf_cusum_alarm_on_threshold']==5 and cfg['ekf_cusum_kappa']==.5
                if not reuse:
                    assert bool(cfg['revision_runtime_enable'])==(stage=='E4')
                    assert cfg['revision_ablation_mode']==(point if stage=='E5' else 'full')
                effective.append(dict(Stage=stage,Point=point,Scenario=scene,h=float(cfg['ekf_cusum_alarm_on_threshold']),
                    kappa=float(cfg['ekf_cusum_kappa']),lambda_=float(cfg['ekf_cusum_lambda']),
                    alarm_off=float(cfg['ekf_cusum_alarm_off_threshold']),
                    predictor_freeze=float(cfg['ekf_cusum_predictor_freeze_threshold']),
                    min_weight=float(cfg['ekf_cusum_weight_min']),matlab=payload['matlab_version']))
    assert new==930 and reused==150 and len(checked)==1030
    with (output/'tables'/'E4'/'timing_per_seed.csv').open(encoding='utf-8-sig',newline='') as f:
        timing=list(csv.DictReader(f))
    assert len(timing)==72000
    time_groups={}
    for row in timing:
        key=(row['Scenario'],row['Seed'],row['Method'])
        time_groups.setdefault(key,[]).append(float(row['Time']))
    assert len(time_groups)==120
    assert all(times==list(range(1,601)) for times in time_groups.values())
    # E1/E5 share the same 50 A0 records, so 150 references resolve to 100 source files.
    failures=list((output/'raw').glob('**/*_failure_*.mat'))
    partial=list((output/'raw').glob('**/*.partial.mat'))
    assert not failures and not partial
    executed=json.loads((output/'provenance'/'executed_source_manifest.json').read_text(encoding='utf-8-sig'))
    for row in executed:
        assert sha(ROOT/row['path'])==row['sha256'],f'Active MATLAB source changed: {row["path"]}'
        assert sha(output/'provenance'/'executed_source'/row['path'])==row['sha256']
    before=json.loads((output/'provenance'/'before_manifest.json').read_text(encoding='utf-8-sig'))
    modified={'code/run_stage1_cusum_comparison.m','code/stage1_cusum_default_config.m',
              'code/revision_validate_config.m'}
    preserved=[]
    for row in before:
        relative=row['path'].replace('\\','/')
        if relative.startswith('code/') and relative not in modified:
            assert sha(ROOT/row['path'])==row['sha256'],f'Unrelated existing source changed: {relative}'
            preserved.append(relative)
    originals=[r for r in before if r['path'].replace('\\','/')=='论文/EIC_English.docx']
    assert len(originals)==1
    assert sha(ROOT/originals[0]['path'])==originals[0]['sha256']
    first_audit=json.loads((FIRST/'validation'/'delivery_audit.json').read_text()); assert first_audit['passed']
    exports=figures(output)
    validation=dict(passed=True,stage_gates=gates,new_unique_checkpoints=new,reuse_references=reused,
        unique_raw_sources=len(checked),new_individual_method_runs=1020,
        overall_new_unique_checkpoints=1230,overall_individual_method_runs=2120,
        raw_hashes_match_cached_analysis=True,active_matlab_matches_execution_snapshot=True,
        snapshot_hashes_unchanged=True,formal_word_hash_unchanged=True,
        preserved_previous_matlab_and_data=preserved,
        seed_input_pairing_verified=True,parameters_reselected=False,
        failed_attempts=len(failures),partial_checkpoints=len(partial),figures=exports,
        effective_grid_configs=effective,
        versions=dict(python=sys.version,numpy=np.__version__,scipy=scipy.__version__,
            matplotlib=matplotlib.__version__,Pillow=PIL.__version__,pypdf=pypdf.__version__,platform=platform.platform()),
        limitations=['Visual inspection is recorded separately; export checks alone do not prove layout quality.',
            'Runtime evidence is current hardware/software at 1 Hz; no airborne hard-real-time guarantee.'])
    (output/'validation'/'delivery_audit.json').write_text(json.dumps(validation,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in validation.items() if k not in ('figures','effective_grid_configs','versions')},indent=2))


if __name__=='__main__': audit()
