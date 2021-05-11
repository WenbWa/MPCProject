% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1)
%   N: MPC horizon length, dimension (1,1)
%   d: Disturbance matrix, dimension (3,N)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_mpc_5(Q,R,T,N,d)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init(Q,R,N);
end

% evaluate control action by solving MPC problem
% [u_mpc,errorcode] = yalmip_optimizer(...,...);
% if (errorcode ~= 0)
%     warning('MPC5 infeasible');
% end
% p = ...; 
[u_mpc,errorcode] = yalmip_optimizer(T, d);
if (errorcode ~= 0)
    warning('MPC5 infeasible');
end
p = u_mpc + param.p_sp;
end

function [param, yalmip_optimizer] = init(Q,R,N)
% get basic controller parameters
% ...
% get terminal cost
% ...
% get terminal set
% ...
% implement your MPC using Yalmip here
% nx = size(param.A,1);
% nu = size(param.B,2);
% U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
% X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
% v = sdpvar(1,1,'full');
% T0 = sdpvar(nx,1,'full');
% d = ...;
% objective = 0;
% constraints = [...];
% for k = 1:N-1
%     constraints = [constraints,...];
%     objective = objective + ...;
% end
% constraints = [constraints, ...];
% objective = objective + ...;
% ops = sdpsettings('verbose',0,'solver','quadprog');
% yalmip_optimizer = optimizer(constraints,objective,ops,...,...);

% get basic controller parameters
param = compute_controller_base_parameters;
yalmip('clear');

% Implement your MPC using Yalmip here
nx = size(param.A,2);
nu = size(param.B,2);
nd = size(param.Bd,2);
U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
error = sdpvar(repmat(2*nx,1,N),ones(1,N),'full');
T0 = sdpvar(nx,1,'full');
D0 = sdpvar(nd,N,'full');
X0 = T0 - param.T_sp;

objective = 0;
constraints = [];
constraints = [constraints, X{1} == X0];
constraints = [constraints, error{1} == zeros(2*nx,1)];
S = diag(ones(2*nx, 1));
v = ones(2*nx, 1);

for k = 1:N-1
    constraints = [constraints, X{k+1} == param.A * X{k} + param.B * U{k} + param.Bd * D0(: ,k)];
    constraints = [constraints, [X{k+1}; -X{k+1}] <= [param.Xcons(:,2); -param.Xcons(:,1)] + error{k+1}];
    constraints = [constraints, error{k+1} >= 0];
    constraints = [constraints, param.Ucons(:,1) <= U{k} <= param.Ucons(:,2)];
    objective = objective + U{k}' * R * U{k} + X{k}' * Q * X{k} + error{k}' * S * error{k} + v' * error{k};
end

[~, P_inf, ~] = dlqr(param.A, param.B, Q, R);
l_f = X{N}' * P_inf * X{N};
objective = objective + l_f + error{N}' * S * error{N} + v' * error{N};

[A_x, b_x] = compute_X_LQR(Q, R);
constraints = [constraints, A_x * X{N} <= b_x];
ops = sdpsettings('verbose', 0, 'solver', 'quadprog');
yalmip_optimizer = optimizer(constraints, objective, ops, {T0,D0}, U{1});

end