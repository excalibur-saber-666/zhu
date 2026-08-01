function results = run_core_tests()
%RUN_CORE_TESTS Run the retained deterministic Stage-1 test suite.

setup_project();
test_names = {
    'test_stage1_cusum_components'
    'test_stage1_3f3l_imu_preint_smoke'
    'test_stage1_four_method_comparison_smoke'};

results = repmat(struct('name', '', 'passed', false, 'elapsed_seconds', NaN, 'error', ''), ...
    numel(test_names), 1);
for index = 1:numel(test_names)
    name = test_names{index};
    started = tic;
    try
        feval(name);
        results(index).passed = true;
    catch exception
        results(index).error = getReport(exception, 'basic', 'hyperlinks', 'off');
    end
    results(index).name = name;
    results(index).elapsed_seconds = toc(started);
    fprintf('%-52s %s (%.3f s)\n', name, local_status(results(index).passed), ...
        results(index).elapsed_seconds);
end

failed = find(~[results.passed]);
if ~isempty(failed)
    failed_names = strjoin({results(failed).name}, ', ');
    error('run_core_tests:Failed', 'One or more core tests failed: %s', failed_names);
end
end

function text_value = local_status(passed)
if passed
    text_value = 'PASS';
else
    text_value = 'FAIL';
end
end
