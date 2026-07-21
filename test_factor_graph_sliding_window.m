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
fprintf('test_factor_graph_sliding_window: PASS\n');
end
