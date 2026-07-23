function matrix = so3_hat(vector)
%SO3_HAT Skew matrix for a three-dimensional rotation vector (rad).
vector = vector(:);
if ~isnumeric(vector) || ~isreal(vector) || numel(vector) ~= 3 || any(~isfinite(vector))
    error('so3_hat:InvalidVector', 'vector must be a finite real 3-vector.');
end
matrix = [0, -vector(3), vector(2); vector(3), 0, -vector(1); -vector(2), vector(1), 0];
end
