% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1)
%   N: MPC horizon length, dimension (1,1)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_mpc_1_forces(Q, R, T, N, ~)
% controller variables
persistent param forces_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, forces_optimizer] = init(Q, R, N);
end

% evaluate control action by solving MPC problem
[u_mpc, errorcode] = forces_optimizer(T);
if errorcode ~= 0
    warning('MPC1_Forces infeasible');
end

p = u_mpc + param.p_sp;
end

function [param, forces_optimizer] = init(Q, R, N)
% get basic controller parameters
% ...
% get terminal cost
% ...
% implement your MPC using Yalmip2Forces interface here
% nx = size(param.A,1);
% nu = size(param.B,2);
% U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
% X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
% T0 = sdpvar(nx,1,'full');
% objective = ...;
% constraints = [...];
% for k = 1:N-1
%     constraints = [constraints,...];
%     objective = objective + ...;
% end
% objective = objective + ...;
% codeoptions = ...;
% forces_optimizer = optimizerFORCES(...);
% get basic controller parameters
param = compute_controller_base_parameters;
yalmip('clear');

% Implement your MPC using Yalmip here
nx = size(param.A,2);
nu = size(param.B,2);
U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
T0 = sdpvar(nx,1,'full');
X0 = T0 - param.T_sp;

objective = 0;
constraints = [];
constraints = [constraints, X{1} == X0];

for k = 1:N-1
    constraints = [constraints, X{k+1} == param.A * X{k} + param.B * U{k}];
    constraints = [constraints, param.Xcons(:,1) <= X{k+1} <= param.Xcons(:,2)];
    constraints = [constraints, param.Ucons(:,1) <= U{k} <= param.Ucons(:,2)];
    objective = objective + U{k}' * R * U{k} + X{k}' * Q * X{k};
end

% Task 10: get terminal cost
[~, P_inf, ~] = dlqr(param.A, param.B, Q, R);
l_f = X{N}' * P_inf * X{N};
objective = objective + l_f;
ops = getOptions('simpleMPC_solver');
forces_optimizer = optimizerFORCES(constraints, objective, ops, T0, U{1}, {'xinit'}, {'u0'});
end
