function y = myMeasurementFcn(x)
%#codegen
% Calculate the accelerometer measurement sensitivity matrix.

% Parameters:
% x: a list representing the state vector, containing the quaternion in
%    the first four elements.
% v: a list representing the velocity vector.

% Returns:
% A 3x4 matrix representing the accelerometer measurement sensitivity matrix.

q = x(1:4);
g = [0, 0, 1];
% Concatenate a 0 to the beginning of the the g vector
g_prime = [0, g];
y_p = quatProd(q, quatProd(g_prime,conj(q)));
y = y_p(2:4);
end

function result = quatProd(q1, q2)
% Multiply two quaternions.

% Parameters:
% q1: a list representing the first quaternion, in the format [w, x, y, z].
% q2: a list representing the second quaternion, in the format [w, x, y, z].

% Returns:
% A list representing the result quaternion, in the format [w, x, y, z].

w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);

w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2;
z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2;

result = [w, x, y, z];
end

function [q_conj] = conj(q)
% Calculate the quaternion conjugate.

% Returns:
% A 4x1 array representing the conjugate of the quaternion.

q_conj = [q(1), -q(2), -q(3), -q(4)];
end