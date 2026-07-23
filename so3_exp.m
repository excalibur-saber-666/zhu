function rotation = so3_exp(rotation_vector)
%SO3_EXP Exponential map from a right SO(3) perturbation in rad to SO(3).
rotation_vector = rotation_vector(:);
if ~isnumeric(rotation_vector) || ~isreal(rotation_vector) || numel(rotation_vector) ~= 3 || ...
        any(~isfinite(rotation_vector))
    error('so3_exp:InvalidVector', 'rotation_vector must be a finite real 3-vector.');
end
angle = norm(rotation_vector);
cross_matrix = so3_hat(rotation_vector);
if angle < 1e-8
    rotation = eye(3) + cross_matrix + 0.5 * cross_matrix * cross_matrix;
else
    rotation = eye(3) + sin(angle) / angle * cross_matrix + ...
        (1 - cos(angle)) / angle^2 * cross_matrix * cross_matrix;
end
rotation = local_project_to_so3(rotation);
end

function rotation = local_project_to_so3(rotation)
[left, ~, right] = svd(rotation);
rotation = left * diag([1, 1, det(left * right')]) * right';
end
