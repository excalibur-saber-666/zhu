function test_revision_experiments(output_directory)
%TEST_REVISION_EXPERIMENTS Baseline equivalence and opt-in scenario checks.
if nargin<1
    output_directory=fullfile(stage1_project_root(),'结果','2026-10-03_E3_E2_revision');
end
before=load(fullfile(output_directory,'validation','before_baseline.mat'),'report');
after=run_stage1_cusum_comparison(before.report.cfg);
names=after.method_names;
for m=1:numel(names)
    a=after.seed_results(1).(names{m}); b=before.report.seed_results(1).(names{m});
    assert(isequal(a.error_xyz,b.error_xyz) && isequal(a.truth_xyz,b.truth_xyz) && ...
        isequaln(a.range_error,b.range_error),'revision:BaselineChanged','Baseline numerical result changed.');
    for k=1:numel(a.history)
        assert(isequaln(a.history(k).detail.final_weight,b.history(k).detail.final_weight) && ...
            isequal(a.history(k).excluded_range_pairs,b.history(k).excluded_range_pairs), ...
            'revision:DetectorChanged','Baseline weights or admission changed.');
    end
end
fprintf('PASS exact pre-change baseline equivalence (all four methods)\n');
cfg=revision_experiment_config('baseline_3f',1001);
cfg.t_stop=40; cfg.fault_segments=[20 25 1 4;30 34 2 5]; cfg.revision_nlos_draw_indices=[1 2];
base=run_stage1_cusum_comparison(cfg);
for scale=[0.5 1.5]
    varied=cfg; varied.revision_nlos_amplitude_scale=scale;
    other=run_stage1_cusum_comparison(varied);
    ac=base.input_caches{1}; bc=other.input_caches{1};
    fields=fieldnames(ac);
    for f=1:numel(fields)
        if ~strcmp(fields{f},'fault_segments')
            assert(isequaln(ac.(fields{f}),bc.(fields{f})),'revision:InputMismatch','Noise changed with intensity.');
        end
    end
    assert(max(abs([bc.fault_segments.bias]-scale*[ac.fault_segments.bias]))<1e-12, ...
        'revision:AmplitudeMismatch','Fault amplitudes are not exact scaled pairs.');
    for m=1:numel(names)
        assert(isequaln(other.seed_results(1).(names{m}).range_error, ...
            other.seed_results(1).ekf.range_error),'revision:MethodInputMismatch','Methods saw different ranges.');
    end
    for k=1:numel(base.seed_results.ekf.history)
        delta=other.seed_results.ekf.history(k).measured_range-base.seed_results.ekf.history(k).measured_range;
        expected=zeros(size(delta)); t=base.seed_results.ekf.range_time(k);
        for j=1:numel(ac.fault_segments)
            event=ac.fault_segments(j);
            if t>=event.start && t<=event.end
                expected(event.edge(1),event.edge(2))=(scale-1)*event.bias;
            end
        end
        assert(max(abs(delta(:)-expected(:)))<1e-10,'revision:InjectedBiasMismatch','Observed scaling differs.');
    end
end
fprintf('PASS actual paired noise/ranges and amplitude-only scaling\n');
for scenario={'size_2f','size_5f'}
    c=revision_experiment_config(scenario{1},1002);
    c.t_stop=5; c.fault_mode='single'; c.fault_enable=false;
    c.revision_nlos_draw_indices=[]; c.methods={'ekf','cusum_fgo'};
    r=run_stage1_cusum_comparison(c); ref=r.seed_results(1).ekf;
    F=c.uav_num-c.high_num;
    p=reshape(ref.truth_xyz(1,:,:),3,F); L=reshape(ref.leader_truth_xyz(1,:,:),3,3);
    for f=1:F
        H=(L-p(:,f))'; ranges=sqrt(sum(H.^2,2)); H=H./ranges;
        assert(all(ranges<500) && rank(H)==3 && cond(H)<10,'revision:BadGeometry','Invalid test formation.');
    end
    assert(size(ref.error_xyz,3)==F && all(isfinite(r.seed_results.cusum_fgo.error_xyz(:))), ...
        'revision:InvalidFormationResult','Formation result failed.');
end
fprintf('PASS 2F3L/5F3L mappings, geometry and finite estimators\n');
c=revision_experiment_config('healthy_maneuver',1002);
c.t_stop=45; c.revision_maneuver_interval=[16 36]; c.fault_mode='single'; c.revision_nlos_draw_indices=[];
r=run_stage1_cusum_comparison(c); ref=r.seed_results.cusum_fgo;
assert(isempty(ref.fault_segments),'revision:UnexpectedFault','Healthy scenario has a fault.');
distance=cat(3,ref.history.true_range);
leader_distance=distance(:,4:6,:);
variation=max(leader_distance,[],3)-min(leader_distance,[],3);
assert(max(variation(:))>1,'revision:IneffectiveMotion','Healthy ranges did not vary.');
fprintf('Short healthy-test leader-range excursion: %.3f m\n',max(variation(:)));
for k=1:numel(ref.history)
    h=ref.history(k); d=h.detail;
    assert(all(isfinite(d.predictor_rate_after)) && ...
        ~any(d.predictor_frozen & d.predictor_level_corrected), ...
        'revision:InvalidPredictorLog','Predictor freeze/update logging is inconsistent.');
end
% The analytical attitude/body-speed derivatives must match finite differences.
t=25; eps_t=1e-4;
[ap,~,vp]=revision_relative_motion(t+eps_t,[0;0;90],zeros(3,1),[0;5;0],zeros(3,1),c);
[am,~,vm]=revision_relative_motion(t-eps_t,[0;0;90],zeros(3,1),[0;5;0],zeros(3,1),c);
[~,rate,~,acc]=revision_relative_motion(t,[0;0;90],zeros(3,1),[0;5;0],zeros(3,1),c);
assert(max(abs((ap(:)-am(:))/(2*eps_t)-rate(:)))<1e-7 && ...
    max(abs((vp(:)-vm(:))/(2*eps_t)-acc(:)))<1e-7, ...
    'revision:InconsistentMotion','Motion derivatives are inconsistent with ideal IMU inputs.');
fprintf('PASS healthy relative motion, ideal-IMU consistency and predictor logs\n');
end
