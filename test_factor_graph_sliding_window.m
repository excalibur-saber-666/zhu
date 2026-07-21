function test_factor_graph_sliding_window()
%TEST_FACTOR_GRAPH_SLIDING_WINDOW Deterministic two-frame graph check.

graph = factor_graph_sliding_window(2, 2);
graph.set_frame_initial(1, [0, 4; 0, 0; 0, 0]);
graph.set_frame_initial(2, [0, 5; 0, 0; 0, 0]);

% Node 1 is a tightly known leader.  Node 2 is a follower constrained by
% ranges and by one metre of SINS relative displacement between the frames.
for frame = 1:2
    graph.add_prior(frame, 1, [0; 0; 0], [0.01; 0.01; 0.01]);
    graph.add_prior(frame, 2, [0; 0; 0], [100; 100; 100]);
end
graph.add_range(1, 1, 2, 5, 0.05, 1);
graph.add_range(2, 1, 2, 6, 0.05, 1);
graph.add_motion(1, 2, 2, [1; 0; 0], [0.05; 0.05; 0.05]);
graph.Gauss_Newton();
graph.covariance();

first_follower = graph.get_position(1, 2);
second_follower = graph.get_position(2, 2);
assert(norm(first_follower - [5; 0; 0]) < 1e-2, ...
    'First-frame range state is incorrect.');
assert(norm(second_follower - [6; 0; 0]) < 1e-2, ...
    'Second-frame range or motion state is incorrect.');
assert(norm((second_follower - first_follower) - [1; 0; 0]) < 1e-2, ...
    'Sliding-window motion factor is incorrect.');
position_covariance = graph.get_position_covariance(2, 2);
assert(all(isfinite(position_covariance(:))) && ...
    norm(position_covariance - position_covariance', 'fro') < 1e-10 && ...
    min(eig(position_covariance)) >= -1e-10, ...
    'Sliding-window position covariance is invalid.');
assert(graph.converged && graph.iteration_count >= 1, ...
    'Sliding-window graph did not converge.');

% A supplied range weight must reach the graph unchanged.  This protects
% the CUSUM-to-window integration from silently reverting to equal weights.
weighted_graph = factor_graph_sliding_window(1, 2);
weighted_graph.set_frame_initial(1, [0, 4; 0, 0; 0, 0]);
weighted_graph.add_prior(1, 1, [0; 0; 0], [0.01; 0.01; 0.01]);
weighted_graph.add_prior(1, 2, [4; 0; 0], [1; 1; 1]);
weighted_graph.add_range(1, 1, 2, 5, 0.05, 0.01);
assert(abs(weighted_graph.range_weights(1) - 0.01) < eps, ...
    'Sliding-window range weight was not retained.');

% A range factor is whitened exactly once: sqrt(weight) / range_std.
% The residual/Jacobian helper used by the window must therefore remain in
% metres; otherwise the Stage-1 sigma would be applied twice.
scale_graph = factor_graph_sliding_window(1, 2);
scale_graph.set_frame_initial(1, [0, 4; 0, 0; 0, 0]);
scale_graph.add_prior(1, 1, [0; 0; 0], [0.01; 0.01; 0.01]);
scale_graph.add_prior(1, 2, [4; 0; 0], [100; 100; 100]);
scale_graph.add_range(1, 1, 2, 5, 0.2, 0.25);
scale_graph.Gauss_Newton();
expected_range_row_norm = sqrt(2) * sqrt(0.25) / 0.2;
assert(abs(norm(scale_graph.A(7, :)) - expected_range_row_norm) < 1e-10, ...
    'Sliding-window range factor is not whitened exactly once.');
fprintf('test_factor_graph_sliding_window: PASS\n');
end
