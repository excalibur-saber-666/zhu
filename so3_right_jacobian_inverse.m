function jacobian_inverse = so3_right_jacobian_inverse(rotation_vector)
%SO3_RIGHT_JACOBIAN_INVERSE Inverse right Jacobian of SO(3), rad input.
rotation_vector = rotation_vector(:);
angle = norm(rotation_vector);
cross_matrix = so3_hat(rotation_vector);
if angle < 1e-8
    jacobian_inverse = eye(3) + 0.5 * cross_matrix + cross_matrix * cross_matrix / 12;
else
    coefficient = 1 / angle^2 - (1 + cos(angle)) / (2 * angle * sin(angle));
    jacobian_inverse = eye(3) + 0.5 * cross_matrix + coefficient * cross_matrix * cross_matrix;
end
end
