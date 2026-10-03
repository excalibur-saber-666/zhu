function run_revision_experiments(stage, seed_list, output_directory)
%RUN_REVISION_EXPERIMENTS Checkpointed E3 then E2, with complete raw records.
% Examples: run_revision_experiments('E3',1:50); after offline E3 acceptance,
% run_revision_experiments('E2',1:50). Disjoint seed lists may run concurrently.
if nargin<2 || isempty(seed_list), seed_list=1:50; end
if nargin<3 || isempty(output_directory)
    output_directory=fullfile(stage1_project_root(),'结果','2026-10-03_E3_E2_revision');
end
assert(isfolder(output_directory),'revision:MissingOutput','Create and freeze the result directory first.');
assert(all(ismember(seed_list,1:50)) && numel(unique(seed_list))==numel(seed_list), ...
    'revision:InvalidSeeds','Formal seeds are fixed to 1:50, without duplicates.');
switch upper(char(stage))
    case 'E3'
        scenarios={'baseline_3f','healthy_maneuver'};
    case 'E2'
        assert(isfile(fullfile(output_directory,'E3_VALIDATED.json')), ...
            'revision:E3NotValidated','Complete and verify E3 before starting E2.');
        scenarios={'size_2f','size_5f','nlos_weak','nlos_strong'};
    otherwise
        error('revision:InvalidStage','Stage must be E3 or E2.');
end
maxNumCompThreads(1);
set(groot,'defaultFigureVisible','off');
for seed=seed_list(:)'
    for scenario_index=1:numel(scenarios)
        scenario=scenarios{scenario_index};
        cfg=revision_experiment_config(scenario,seed);
        folder=fullfile(output_directory,'raw',scenario);
        if ~isfolder(folder), mkdir(folder); end
        filename=fullfile(folder,sprintf('seed_%04d.mat',seed));
        if isfile(filename)
            existing=load(filename,'payload');
            assert(isfield(existing,'payload') && existing.payload.complete && ...
                isequaln(existing.payload.cfg,cfg),'revision:ExistingMismatch', ...
                'Existing checkpoint does not match the frozen configuration.');
            clear existing;
            fprintf('REUSE %s seed=%d\n',scenario,seed);
            continue;
        end
        timer=tic;
        try
            report=run_stage1_cusum_comparison(cfg);
            current=report.seed_results(1);
            reference=current.(report.method_names{1});
            payload=struct();
            payload.schema_version=1; payload.complete=false;
            payload.scenario=scenario; payload.seed=seed; payload.cfg=report.cfg;
            payload.time=reference.time; payload.range_time=reference.range_time;
            payload.truth_xyz=reference.truth_xyz; payload.leader_truth_xyz=reference.leader_truth_xyz;
            payload.input_cache=report.input_caches{1};
            payload.fault_segments=reference.fault_segments;
            payload.method_names=report.method_names;
            payload.methods=struct();
            for method_index=1:numel(report.method_names)
                name=report.method_names{method_index}; result=current.(name);
                assert(isequal(result.truth_xyz,reference.truth_xyz) && ...
                    isequal(result.leader_truth_xyz,reference.leader_truth_xyz) && ...
                    isequaln(result.range_error,reference.range_error), ...
                    'revision:PairingFailed','Actual method trajectory/ranges are not identical.');
                assert(all(isfinite(result.error_xyz(:))), ...
                    'revision:NonfiniteResult','Nonfinite navigation result must be reported as a failure.');
                for k=1:numel(result.history)
                    assert(isequaln(result.history(k).graph_prior_positions,reference.history(k).graph_prior_positions), ...
                        'revision:GPSMismatch','Actual GPS priors differ across methods.');
                end
                result=rmfield(result,{'time','range_time','truth_xyz','leader_truth_xyz', ...
                    'navigation_xyz','final_cusum_state'});
                payload.methods.(name)=result;
            end
            payload.elapsed_seconds=toc(timer);
            payload.matlab_version=version;
            payload.complete=true;
            temporary=[filename '.partial.mat'];
            save(temporary,'payload','-v7');
            movefile(temporary,filename);
            fprintf('DONE %s seed=%d seconds=%.3f file=%s\n',scenario,seed,toc(timer),filename);
            clear payload report current reference result;
        catch err
            failure=struct('scenario',scenario,'seed',seed,'cfg',cfg, ...
                'message',getReport(err,'extended','hyperlinks','off'),'elapsed_seconds',toc(timer));
            save(fullfile(folder,sprintf('seed_%04d_failure_%s.mat',seed, ...
                char(datetime('now','Format','yyyyMMdd_HHmmss')))),'failure','-v7');
            fprintf(2,'FAILED %s seed=%d %s\n',scenario,seed,failure.message);
        end
    end
end
fprintf('WORKER_FINISHED %s seeds=%s\n',upper(char(stage)),mat2str(seed_list));
end
