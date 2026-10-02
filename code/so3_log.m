function rotation_vector = so3_log(rotation)
%SO3_LOG Logarithm map from SO(3) to a right rotation vector in rad.
if ~isnumeric(rotation) || ~isreal(rotation) || ~isequal(size(rotation), [3, 3]) || ...
        any(~isfinite(rotation(:)))
    error('so3_log:InvalidRotation', 'rotation must be a finite real 3-by-3 matrix.');
end
orthogonality_error = norm(rotation' * rotation - eye(3), 'fro');
determinant_error = abs(det(rotation) - 1);
if orthogonality_error > 1e-10 || determinant_error > 1e-10
    [left, ~, right] = svd(rotation);
    rotation = left * diag([1, 1, det(left * right')]) * right';
end
% The project already contains trace.m for trajectory generation, so do not
% call MATLAB's shadowed trace() here.
cosine = min(1, max(-1, (sum(diag(rotation)) - 1) / 2));
angle = acos(cosine);
vee = [rotation(3, 2) - rotation(2, 3); ...
       rotation(1, 3) - rotation(3, 1); ...
       rotation(2, 1) - rotation(1, 2)];
if angle < 1e-8
    rotation_vector = 0.5 * vee;
elseif pi - angle < 1e-5
    axis = sqrt(max(0, (diag(rotation) + 1) / 2));
    [~, index] = max(axis);
    if axis(index) < 1e-8
        axis = [1; 0; 0];
    else
        remaining = setdiff(1:3, index);
        axis(remaining) = (rotation(remaining, index) + rotation(index, remaining)) / (4 * axis(index));
    end
    rotation_vector = angle * axis / norm(axis);
else
    rotation_vector = angle / (2 * sin(angle)) * vee;
end
end
