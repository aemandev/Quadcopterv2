function [x] = myStateTransitionFcn(x,w_body)
%#codegen

% x: State vector; [q, w]
% dt: Time step
dt = .01;
% Does not even have to be the same dim as x0
q = x(1:4);

% Calculate angle and axis
q_delta = vec2quat(w_body, dt);

q_disturbed = quatProd(q,q_delta);
w_disturbed = x(5:7) + w_body';

q_disturbed = q_disturbed/norm(q_disturbed);
x = [q_disturbed'; w_disturbed];

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

function q_new = vec2quat(vec, dt)
% Convert a 3D rotation vector to quaternion.

% Parameters:
% vec: a list representing a 3D rotation vector.
% dt: the time step (optional).

% Returns:
% A list representing the quaternion, in the format [w, x, y, z].

if nargin == 1
    if ~any(vec)
        q_new = [1, 0, 0, 0];
    else
        a = norm(vec);
        e = vec / norm(vec) * sin(a / 2);
        q_new = [cos(a / 2), e(1), e(2), e(3)];
    end
else
    if ~any(vec)
        q_new = [1, 0, 0, 0];
    else
        a = norm(vec) * dt;
        e = vec / norm(vec) * sin(a / 2);
        q_new = [cos(a / 2), e(1), e(2), e(3)];
    end
end
q_new = q_new / norm(q_new);

end