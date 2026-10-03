"""Hand-calculated statistical cases; these are not Monte Carlo observations."""
import unittest
import math
from analyze_revision_experiments import event_metrics, position_metrics, build_trace
import numpy as np


class RevisionStatisticsTest(unittest.TestCase):
    def setUp(self):
        self.t = np.arange(1,51,dtype=float)
        self.observed = np.ones(50,dtype=bool)
        self.threshold = (self.t>=5)&(self.t<=7)
        self.alarm = (self.t>=6)&(self.t<=7)
        self.isolated = self.alarm.copy()

    def evaluate(self, **kw):
        values=dict(times=self.t,start=4,end=6,threshold=self.threshold,alarm=self.alarm,
                    isolated=self.isolated,observed=self.observed)
        values.update(kw)
        return event_metrics(**values)

    def test_distinct_delays_and_early_known_recovery(self):
        r=self.evaluate()
        self.assertEqual((r['DetectionDelay'],r['AlarmDelay'],r['IsolationDelay']),(1,2,2))
        self.assertEqual((r['RecoveryTime'],r['RecoveryDelay'],r['RecoveryConfirmTime']),(8,2,10))
        # A confirmed success does not become censored merely because the run later ends before the horizon.
        r=self.evaluate(times=self.t[:20],threshold=self.threshold[:20],alarm=self.alarm[:20],
                        isolated=self.isolated[:20],observed=self.observed[:20])
        self.assertEqual(r['RecoveryStatus'],'recovered')

    def test_missed_is_not_zero_delay_or_recovery_success(self):
        off=np.zeros(50,dtype=bool)
        r=self.evaluate(threshold=off,alarm=off,isolated=off)
        self.assertEqual(r['DetectionStatus'],'missed')
        self.assertTrue(math.isnan(r['DetectionDelay']))
        self.assertEqual(r['RecoveryStatus'],'not_applicable')

    def test_full_observation_failure(self):
        r=self.evaluate(isolated=self.t>=6)
        self.assertEqual(r['RecoveryStatus'],'failed')
        self.assertTrue(math.isnan(r['RecoveryDelay']))

    def test_admission_at_horizon_requires_later_confirmation(self):
        # Observed in formal seed 30/event 8: first admission at end + 30
        # is not a stable recovery confirmed by that deadline.
        state=(self.t>=6)&(self.t<36)
        self.assertEqual(self.evaluate(isolated=state)['RecoveryStatus'],'failed')
        later=self.evaluate(isolated=state,recovery_horizon=40)
        self.assertEqual((later['RecoveryDelay'],later['RecoveryConfirmTime']),(30,38))

    def test_short_observation_is_censored(self):
        r=self.evaluate(times=self.t[:20],threshold=self.threshold[:20],alarm=self.alarm[:20],
                        isolated=self.t[:20]>=6,observed=self.observed[:20])
        self.assertEqual((r['RecoveryStatus'],r['CensorReason']),('censored','run_ended'))

    def test_next_event_censors_unresolved_episode(self):
        r=self.evaluate(isolated=self.t>=6,next_start=12)
        self.assertEqual((r['RecoveryStatus'],r['CensorReason']),('censored','next_same_edge_fault'))

    def test_missing_edge_censors_unresolved_episode(self):
        observed=self.observed.copy(); observed[7]=False
        r=self.evaluate(observed=observed)
        self.assertEqual((r['RecoveryStatus'],r['CensorReason']),('censored','edge_unobservable'))

    def test_persistent_episode_not_counted_as_fresh_second_detection(self):
        state=self.t>=2
        r=self.evaluate(start=10,end=12,threshold=state,alarm=state,isolated=state)
        self.assertEqual(r['DetectionStatus'],'preexisting')
        self.assertEqual(r['IsolationStatus'],'preexisting')
        self.assertTrue(math.isnan(r['IsolationDelay']))

    def test_macro_metrics_have_distinct_meanings(self):
        error=np.zeros((2,3,2)); error[:,0,0]=[1,3]; error[:,0,1]=[4,4]
        rows=position_metrics(error)
        macro=next(r for r in rows if r['Aggregation']=='macro')
        pooled=next(r for r in rows if r['Aggregation']=='pooled')
        self.assertEqual(macro['Mean'],3)
        self.assertEqual(macro['Mean'],pooled['Mean'])
        self.assertEqual((macro['CDF95'],pooled['CDF95']),(3.5,4))
        self.assertEqual((macro['Maximum'],pooled['Maximum']),(3.5,4))
        self.assertAlmostEqual(macro['RMSE3D'],(math.sqrt(5)+4)/2)
        self.assertAlmostEqual(pooled['RMSE3D'],math.sqrt(10.5))

    def test_historical_factors_age_out_after_admission_stops(self):
        history=[]
        for t in range(1,13):
            admitted=t<5 or t>=10
            history.append(dict(time=t,pairs=np.array([[1,2]]),
                admitted_range_pairs=np.array([[1,2]]) if admitted else np.empty((0,2)),
                true_range=np.array([[0,100,110,120]]),detail={}))
        p=dict(cfg=dict(uav_num=4,high_num=3,sliding_window_length=3),range_time=np.arange(1,13),
               methods=dict(cusum_fgo=dict(history=history)),
               fault_segments=[dict(start=3,end=4,edge=[1,2],bias=2)])
        tr=build_trace(p,'cusum_fgo')
        self.assertEqual(list(tr['retained_fault_factors'][2:7,0]),[1,2,2,1,0])
        self.assertEqual(list(tr['retained_factors'][3:7,0]),[3,2,1,0])
        self.assertTrue(tr['isolated'][4,0])


if __name__=='__main__':
    unittest.main(verbosity=2)
