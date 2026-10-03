"""Hand-computable checks for point-specific thresholds and online time accounting."""
import unittest
import math
from analyze_remaining_revision import threshold_flags, runtime_rows, expected_jobs
from analyze_revision_experiments import event_metrics
import numpy as np


class RemainingAnalysisTest(unittest.TestCase):
    def test_threshold_grid_changes_detection_epoch(self):
        times=np.arange(1,7); plus=np.array([0,2,4,6,8,0]); off=np.zeros(6,bool)
        for h,expected in ((3,1),(5,2),(7,3)):
            state=threshold_flags(plus,np.zeros(6),h)
            event=event_metrics(times,2,5,state,off,off,np.ones(6,bool))
            self.assertEqual(event['DetectionDelay'],expected)
        self.assertTrue(threshold_flags(np.array([0]),np.array([5]),5)[0])

    def test_online_budget_includes_sins_and_keyframe(self):
        history=[]
        for t in range(1,11):
            algorithm=t*.001; keyframe=2*algorithm
            history.append(dict(time=t,algorithm_specific_seconds=algorithm,
                online_keyframe_seconds=keyframe,sins_interval_seconds=.003,
                end_to_end_online_seconds=keyframe+.003,sins_interval_steps=50,
                graph_solve_seconds=.0001,gn_iteration_count=2,gn_final_step_norm=1e-6,gn_converged=True))
        p={'methods':{'cusum_fgo':{'history':history},'ekf':{'history':history}}}
        rows,summary=runtime_rows(p,'cusum_fgo',{})
        full=next(r for r in summary if r['Phase']=='all')
        steady=next(r for r in summary if r['Phase']=='steady')
        self.assertAlmostEqual(full['AlgorithmSeconds_Mean'],.0055)
        self.assertAlmostEqual(full['EndToEndSeconds_Mean'],.014)
        self.assertAlmostEqual(steady['EndToEndSeconds_Max'],.023)
        self.assertEqual((full['GNCapHits'],full['GNUnconverged']),(0,0))
        _,ekf=runtime_rows(p,'ekf',{})
        self.assertTrue(math.isnan(ekf[0]['GNMean']))

    def test_frozen_counts(self):
        self.assertEqual(len(list(expected_jobs('E1'))),900)
        self.assertEqual(len(list(expected_jobs('E4'))),30)
        self.assertEqual(len(list(expected_jobs('E5'))),150)


if __name__=='__main__': unittest.main(verbosity=2)
