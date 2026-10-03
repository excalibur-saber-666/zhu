"""Offline explanations of actual E3 failures; does not change the 30 s endpoint."""
import csv
import math
from analyze_revision_experiments import DEFAULT_OUTPUT, write_csv, event_metrics, rising
import numpy as np


def main(output=DEFAULT_OUTPUT):
    with (output/'tables'/'E3'/'events_per_seed.csv').open(encoding='utf-8-sig') as f:
        events=[r for r in csv.DictReader(f) if r['Method']=='cusum_fgo']
    failure_rows=[]; missed=[]; healthy_episodes=[]
    for seed in range(1,51):
        tr=dict(np.load(output/'derived'/'baseline_3f'/f'seed_{seed:04d}_cusum_fgo_trace.npz'))
        current=[e for e in events if int(e['Seed'])==seed]
        for e in current:
            pair=np.array([int(e['Follower']),3+int(e['Leader'])]); j=np.flatnonzero(np.all(tr['pairs']==pair,axis=1))[0]
            start=float(e['FaultStart']); end=float(e['FaultEnd']); t=tr['time']
            active=(t>=start)&(t<=end)
            if e['RecoveryStatus']=='failed':
                later=[float(other['FaultStart']) for other in current if
                    other['Follower']==e['Follower'] and other['Leader']==e['Leader'] and float(other['FaultStart'])>end]
                next_start=min(later,default=math.inf)
                extended=event_metrics(t,start,end,tr['threshold'][:,j],tr['alarm'][:,j],
                    tr['isolated'][:,j],tr['observed'][:,j],next_start=next_start,recovery_horizon=600)
                mask=(t>end)&(t<=end+30)
                failure_rows.append(dict(Seed=seed,Event=int(e['Event']),Follower=pair[0],Leader=pair[1]-3,
                    FaultEnd=end,FormalRecoveryStatus='failed_within_30s',
                    LaterRecoveryStatus=extended['RecoveryStatus'],LaterRecoveryDelay=extended['RecoveryDelay'],
                    LaterObservationStop=extended['CensorReason'],
                    MedianAbsInnovation30s=float(np.median(np.abs(tr['z'][mask,j]))),
                    MedianPredictionError30s=float(np.median(tr['prediction'][mask,j]-tr['true_range'][mask,j])),
                    PredictorFrozenFraction30s=float(np.mean(tr['frozen'][mask,j])),
                    AlarmFraction30s=float(np.mean(tr['alarm'][mask,j])),
                    IsolationFraction30s=float(np.mean(tr['isolated'][mask,j]))))
            if e['IsolationStatus']=='missed':
                candidates=(tr['pairs'][:,0]==pair[0])&(tr['pairs'][:,1]>3)
                competitors=[]
                for k in np.flatnonzero(candidates):
                    if k!=j and np.any(active & tr['isolated'][:,k]):
                        competitors.append(f"F{tr['pairs'][k,0]}-L{tr['pairs'][k,1]-3}")
                missed.append(dict(Seed=seed,Event=int(e['Event']),Follower=pair[0],Leader=pair[1]-3,
                    DetectionDelay=e['DetectionDelay'],AlarmDelay=e['AlarmDelay'],
                    TargetAlarmFraction=float(np.mean(tr['alarm'][active,j])),
                    CompetitorIsolation=';'.join(competitors)))
        healthy=dict(np.load(output/'derived'/'healthy_maneuver'/f'seed_{seed:04d}_cusum_fgo_trace.npz'))
        for j,pair in enumerate(healthy['pairs']):
            if pair[1]<=3: continue
            state=healthy['isolated'][:,j]
            for first in np.flatnonzero(rising(state,healthy['observed'][:,j])):
                end_candidates=np.flatnonzero(~state[first:])
                final=first+end_candidates[0]-1 if len(end_candidates) else len(state)-1
                active=slice(first,final+1)
                healthy_episodes.append(dict(Seed=seed,Follower=int(pair[0]),Leader=int(pair[1]-3),
                    Start=float(healthy['time'][first]),LastIsolatedEpoch=float(healthy['time'][final]),
                    OccupiedEpochs=final-first+1,StillIsolatedAtRunEnd=final==len(state)-1,
                    MedianAbsInnovation=float(np.median(np.abs(healthy['z'][active,j]))),
                    MedianPredictionError=float(np.median(healthy['prediction'][active,j]-healthy['true_range'][active,j])),
                    PredictorFrozenFraction=float(np.mean(healthy['frozen'][active,j]))))
    write_csv(output/'tables'/'E3'/'recovery_failure_diagnostics.csv',failure_rows)
    write_csv(output/'tables'/'E3'/'missed_isolation_diagnostics.csv',missed)
    write_csv(output/'tables'/'E3'/'healthy_isolation_episodes.csv',healthy_episodes)
    print('Recovery failures:',len(failure_rows),'later recovered:',sum(r['LaterRecoveryStatus']=='recovered' for r in failure_rows))
    print('Missed isolation:',missed)
    print('Healthy episodes:',len(healthy_episodes),'persisted at end:',sum(r['StillIsolatedAtRunEnd'] for r in healthy_episodes))
    print('First recovery failure:',failure_rows[0] if failure_rows else None)


if __name__=='__main__': main()
