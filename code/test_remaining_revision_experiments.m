function test_remaining_revision_experiments(output_directory)
%TEST_REMAINING_REVISION_EXPERIMENTS Numeric invariance, clocks and ablation semantics.
if nargin<1
    output_directory=fullfile(stage1_project_root(),'结果','2026-10-03_E1_E4_E5_revision');
end
first=fullfile(stage1_project_root(),'结果','2026-10-03_E3_E2_revision');
before=load(fullfile(first,'validation','before_baseline.mat'),'report');
plain=run_stage1_cusum_comparison(before.report.cfg);
timed_cfg=before.report.cfg; timed_cfg.revision_runtime_enable=true;
timed=run_stage1_cusum_comparison(timed_cfg);
for name=plain.method_names
    a=plain.seed_results.(name{1}); b=before.report.seed_results.(name{1}); c=timed.seed_results.(name{1});
    assert(isequal(a.error_xyz,b.error_xyz) && isequal(a.error_xyz,c.error_xyz), ...
        'revision:InstrumentationChangedResults','Opt-in instrumentation changed numerical baseline.');
    assert(all(isnan([a.history.end_to_end_online_seconds])));
    for k=1:numel(c.history)
        row=c.history(k);
        assert(row.sins_interval_steps==50 && row.algorithm_specific_seconds>0 && ...
            row.online_keyframe_seconds>=row.algorithm_specific_seconds && row.sins_interval_seconds>0 && ...
            abs(row.end_to_end_online_seconds-row.sins_interval_seconds-row.online_keyframe_seconds)<1e-12);
        assert(isequal(a.history(k).admitted_range_pairs,c.history(k).admitted_range_pairs));
    end
end
fprintf('PASS all four methods: exact baseline and timer invariance, full 50-step online intervals\n');
cfg=remaining_revision_config('E5','baseline_3f',1002);
cfg.t_stop=40; cfg.fault_segments=[20 25 1 4]; cfg.revision_nlos_draw_indices=1;
full=run_stage1_cusum_comparison(cfg); base=full.seed_results.cusum_fgo;
assert(any(arrayfun(@(h)~isempty(h.excluded_range_pairs),base.history)));
assert(any(arrayfun(@(h)any(h.detail.final_weight<1),base.history)));
fields={'signed_normalized_innovation','cusum_positive','cusum_negative','alarm_active', ...
        'predictor_frozen','predictor_value_after'};
for mode={'no_isolation','no_soft_weighting'}
    varied=cfg; varied.revision_ablation_mode=mode{1};
    other=run_stage1_cusum_comparison(varied); trace=other.seed_results.cusum_fgo;
    assert(isequaln(trace.range_error,base.range_error));
    for k=1:numel(base.history)
        a=base.history(k); b=trace.history(k);
        for f=1:numel(fields)
            assert(isequaln(a.detail.(fields{f}),b.detail.(fields{f})), ...
                'revision:AblationDisabledDetector','Ablation unintentionally changed the detector.');
        end
        if strcmp(mode{1},'no_isolation')
            assert(isempty(b.excluded_range_pairs) && isequal(a.detail.final_weight,b.detail.final_weight));
        else
            assert(all(b.detail.final_weight==1) && isequal(a.excluded_range_pairs,b.excluded_range_pairs));
        end
    end
end
fprintf('PASS isolated mechanism switches: unchanged detector, intended admission/weights only\n');
for h=[3 5 7]
    for kappa=[0.25 0.5 0.75]
        point=remaining_revision_config('E1','healthy_maneuver',1002,h,kappa);
        assert(point.ekf_cusum_alarm_on_threshold==h && point.ekf_cusum_kappa==kappa && ...
            ~point.fault_enable && ~point.revision_runtime_enable && strcmp(point.revision_ablation_mode,'full'));
    end
end
save(fullfile(output_directory,'validation','short_validation.mat'),'plain','timed','full','-v7');
fprintf('PASS E1 grid and preserved no-NLOS relative-maneuver configuration\n');
end
