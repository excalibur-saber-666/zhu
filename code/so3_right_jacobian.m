function jacobian = so3_right_jacobian(rotation_vector)
%SO3_RIGHT_JACOBIAN Right Jacobian of SO(3) for a vector in rad.
rotation_vector = rotation_vector(:);
angle = norm(rotation_vector);
cross_matrix = so3_hat(rotation_vector);
if angle < 1e-8
    jacobian = eye(3) - 0.5 * cross_matrix + cross_matrix * cross_matrix / 6;
else
    jacobian = eye(3) - (1 - cos(angle)) / angle^2 * cross_matrix + ...
        (angle - sin(angle)) / angle^3 * cross_matrix * cross_matrix;
end
end
