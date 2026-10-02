function [residual, jacobian] = range_residual_cal(position_i, position_j, measurement)
%RANGE_RESIDUAL_CAL Unwhitened Euclidean range residual and Jacobian.
%   The caller applies its own measurement standard deviation and any
%   robust/CUSUM weight.  Keeping this residual in metres avoids applying
%   the range standard deviation twice in the sliding-window graph.

displacement = position_i - position_j;
predicted_range = norm(displacement);
if ~isfinite(predicted_range) || predicted_range <= eps
    error('range_residual_cal:DegenerateRange', ...
        'The two range nodes must have a finite nonzero separation.');
end
unit_direction = displacement / predicted_range;
residual = predicted_range - measurement;
jacobian = [unit_direction', -unit_direction'];
end
