clear all
acc_vals = [1,2,3];
n       = 6;
n_data  = size(acc_vals, 2);
q       = zeros(n_data,4);
dcm    = zeros(3, 3, n_data);
dt  = .01;
% dt = ones(size(ts,1), size(ts,2))*0.01;
thresh  = 1e-3;      
max_iter = 100;
P = .01*eye(6);
Q = .001*eye(6);


s = [1,0,0,0,0,0,0];   % the initial state in vector space
g = [0,0,0,1];

    
% compute dq from omega and dt
%     w = gyro_vals(:,i)';
w= [.1, .1,.1];
w_norm = sqrt(sum(w.^2));
angle = w_norm*.01;
axis = w/w_norm;
dq = [cos(angle/2) axis.*sin(angle/2)];
dq(isnan(dq)) = 0;

% compute sigma points

S = chol(P+Q);          % S 6 * 6
S = sqrt(2*n)*S;        
W = [S, -S];            % W 6 * 12

% convert 6d W into 7d sigma points X: 12*7
X(:, 1:4) = vec2quat(W(1:3,:));
X(:, 5:7) = W(4:6,:)';
X(:, 1:4) = quatmultiply(s(1:4), X(:,1:4));

Y(:,1:4) = quatmultiply(X(:,1:4),dq);
Y(:,5:7) = bsxfun(@plus, X(:,5:7), w);
a = 5


function q = vec2quat(vec)
% vec   n*3
% q     n*4
    angle = sqrt(sum(vec(1:3,:).^2,1));
    ev = bsxfun(@rdivide, vec(1:3,:), angle);
    q = [cos(angle/2); bsxfun(@times, ev, sin(angle/2))]';
    q(isnan(q)) = 0;
    q(isinf(q)) = 0;
end

function vec = quat2vec(q)
% vec   n*3
% q     n*4
    
    q = quatnormalize(q)';
    angles = acos(q(1,:))*2; % 1*n;
    sins = sin(angles); % 1*n
    vec = bsxfun(@times, (bsxfun(@rdivide, q(2:4,:), sins))', sins')';
%     vec = bsxfun(@times, (bsxfun(@rdivide, q(2:4,:), sqrt(1 - q(1,:).^2)))', acos(q(1,:))'*2)';
    vec(isnan(vec)) = 0;
    vec(isinf(vec)) = 0;
end