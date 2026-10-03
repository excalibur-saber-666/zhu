function run_remaining_revision_experiments(stage, seeds, output_directory)
%RUN_REMAINING_REVISION_EXPERIMENTS Execute E1, then E4, then E5 checkpoints.
if nargin<2, seeds=1:50; end
if nargin<3
    output_directory=fullfile(stage1_project_root(),'结果','2026-10-03_E1_E4_E5_revision');
end
stage=upper(char(stage));
assert(isfile(fullfile(output_directory,'frozen_manifest.json')), ...
    'revision:NotFrozen','Freeze all definitions before formal execution.');
if strcmp(stage,'E4'), required_gate='E1_VALIDATED.json';
elseif strcmp(stage,'E5'), required_gate='E4_VALIDATED.json';
else, required_gate=''; end
if ~isempty(required_gate)
    assert(isfile(fullfile(output_directory,required_gate)), ...
        'revision:PreviousStageIncomplete','Verify the preceding stage first.');
end
allowed=1:50;
if strcmp(stage,'E4'), allowed=1:10; end
assert(all(ismember(seeds,allowed)) && numel(unique(seeds))==numel(seeds));
maxNumCompThreads(1); set(groot,'defaultFigureVisible','off');
if strcmp(stage,'E4')
    % Warm all methods/sizes once; these seeds and short traces are not data.
    for scene={'baseline_3f','size_2f','size_5f'}
        cfg=remaining_revision_config('E4',scene{1},1003);
        cfg.t_stop=30; cfg.fault_segments=[20 25 1 cfg.uav_num-cfg.high_num+1];
        cfg.revision_nlos_draw_indices=1;
        run_stage1_cusum_comparison(cfg);
    end
    fprintf('WARMUP_COMPLETE E4 seed=1003 duration=30\n');
end
for seed=seeds(:)'
    jobs=local_jobs(stage,seed);
    for j=1:numel(jobs)
        job=jobs(j); cfg=job.cfg;
        folder=fullfile(output_directory,'raw',stage,job.id,job.scenario);
        if ~isfolder(folder), mkdir(folder); end
        filename=fullfile(folder,sprintf('seed_%04d.mat',seed));
        reuse_file=fullfile(folder,sprintf('seed_%04d.reuse.json',seed));
        if job.reuse
            source_relative=fullfile('结果','2026-10-03_E3_E2_revision','raw',job.scenario,sprintf('seed_%04d.mat',seed));
            assert(isfile(fullfile(stage1_project_root(),source_relative)));
            reuse=struct('stage',stage,'point',job.id,'scenario',job.scenario,'seed',seed, ...
                'source_relative_to_project',source_relative,'method','cusum_fgo','reason','unchanged_center_or_A0');
            fid=fopen(reuse_file,'w','n','UTF-8'); assert(fid>=0);
            fwrite(fid,jsonencode(reuse),'char'); fclose(fid);
            fprintf('REUSE %s %s %s seed=%d\n',stage,job.id,job.scenario,seed);
            continue;
        end
        if isfile(filename)
            saved=load(filename,'payload');
            assert(saved.payload.complete && isequaln(saved.payload.cfg,cfg), ...
                'revision:CheckpointMismatch','Existing checkpoint differs from frozen config.');
            clear saved; fprintf('EXISTS %s %s seed=%d\n',stage,job.id,seed); continue;
        end
        timer=tic;
        try
            report=run_stage1_cusum_comparison(cfg);
            current=report.seed_results(1);
            reference=current.(report.method_names{1});
            payload=struct('schema_version',1,'complete',false,'stage',stage,'point',job.id, ...
                'scenario',job.scenario,'seed',seed,'cfg',report.cfg,'method_names',{report.method_names}, ...
                'time',reference.time,'range_time',reference.range_time,'truth_xyz',reference.truth_xyz, ...
                'leader_truth_xyz',reference.leader_truth_xyz,'input_cache',report.input_caches{1}, ...
                'fault_segments',reference.fault_segments,'methods',struct(),'matlab_version',version);
            for m=1:numel(report.method_names)
                name=report.method_names{m}; result=current.(name);
                assert(isequal(result.truth_xyz,reference.truth_xyz) && ...
                    isequaln(result.range_error,reference.range_error));
                assert(all(isfinite(result.error_xyz(:))));
                for k=1:numel(result.history)
                    assert(isequaln(result.history(k).measured_range,reference.history(k).measured_range) && ...
                        isequal(result.history(k).graph_prior_positions,reference.history(k).graph_prior_positions));
                end
                result=rmfield(result,{'time','range_time','truth_xyz','leader_truth_xyz','navigation_xyz','final_cusum_state'});
                payload.methods.(name)=result;
            end
            payload.elapsed_seconds=toc(timer); payload.complete=true;
            temporary=[filename '.partial.mat']; save(temporary,'payload','-v7'); movefile(temporary,filename);
            fprintf('DONE %s %s %s seed=%d seconds=%.3f\n',stage,job.id,job.scenario,seed,toc(timer));
            clear payload report current reference result;
        catch err
            failure=struct('stage',stage,'point',job.id,'scenario',job.scenario,'seed',seed,'cfg',cfg, ...
                'message',getReport(err,'extended','hyperlinks','off'),'elapsed_seconds',toc(timer));
            save(fullfile(folder,sprintf('seed_%04d_failure_%s.mat',seed,char(datetime('now','Format','yyyyMMdd_HHmmss')))), ...
                'failure','-v7');
            fprintf(2,'FAILED %s %s seed=%d %s\n',stage,job.id,seed,failure.message);
        end
    end
end
fprintf('WORKER_FINISHED %s seeds=%s\n',stage,mat2str(seeds));
end

function jobs=local_jobs(stage,seed)
jobs=repmat(struct('id','','scenario','','cfg',struct(),'reuse',false),0,1);
switch stage
    case 'E1'
        for h=[3 5 7]
            for kappa=[0.25 0.5 0.75]
                id=strrep(sprintf('h%g_k%g',h,kappa),'.','p');
                for scene={'baseline_3f','healthy_maneuver'}
                    jobs(end+1)=struct('id',id,'scenario',scene{1}, ...
                        'cfg',remaining_revision_config(stage,scene{1},seed,h,kappa), ...
                        'reuse',h==5 && kappa==0.5); %#ok<AGROW>
                end
            end
        end
    case 'E4'
        for scene={'baseline_3f','size_2f','size_5f'}
            jobs(end+1)=struct('id','runtime','scenario',scene{1}, ...
                'cfg',remaining_revision_config(stage,scene{1},seed),'reuse',false); %#ok<AGROW>
        end
    case 'E5'
        for variant={'full','no_isolation','no_soft_weighting'}
            jobs(end+1)=struct('id',variant{1},'scenario','baseline_3f', ...
                'cfg',remaining_revision_config(stage,'baseline_3f',seed,5,0.5,variant{1}), ...
                'reuse',strcmp(variant{1},'full')); %#ok<AGROW>
        end
    otherwise
        error('revision:UnknownRemainingStage','Unknown stage.');
end
end
