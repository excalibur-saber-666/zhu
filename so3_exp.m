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
% Rodrigues' formula already produces an SO(3) matrix to round-off.  Avoid an
% SVD projection here because this function is called in every numerical
% Jacobian evaluation; callers that receive externally supplied matrices still
% validate/project them at their boundary.
end
