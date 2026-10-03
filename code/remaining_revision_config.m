function cfg = remaining_revision_config(stage, scenario, seed, h, kappa, ablation)
%REMAINING_REVISION_CONFIG Frozen E1/E4/E5 choices; no evaluation-based tuning.
if nargin<4, h=5; end
if nargin<5, kappa=0.5; end
if nargin<6, ablation='full'; end
cfg=revision_experiment_config(scenario,seed);
switch upper(char(stage))
    case 'E1'
        assert(ismember(h,[3 5 7]) && ismember(kappa,[0.25 0.5 0.75]), ...
            'revision:GridChanged','E1 grid was frozen before formal runs.');
        assert(ismember(char(scenario),{'baseline_3f','healthy_maneuver'}));
        cfg.methods={'cusum_fgo'};
        cfg.ekf_cusum_alarm_on_threshold=h;
        cfg.ekf_cusum_kappa=kappa;
    case 'E4'
        assert(ismember(char(scenario),{'baseline_3f','size_2f','size_5f'}));
        cfg.revision_runtime_enable=true;
        % Rotate method order; no RNG draw is consumed by this policy.
        methods={'ekf','fgo','cusum_ekf','cusum_fgo'};
        cfg.methods=circshift(methods,[0 mod(seed-1,4)]);
    case 'E5'
        assert(strcmp(char(scenario),'baseline_3f') && ...
            ismember(char(ablation),{'full','no_isolation','no_soft_weighting'}));
        cfg.methods={'cusum_fgo'};
        cfg.revision_ablation_mode=char(ablation);
    otherwise
        error('revision:UnknownRemainingStage','Unknown stage.');
end
revision_validate_config(cfg);
end
