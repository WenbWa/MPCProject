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

function p = controller_mpc_6(Q,R,T,N,~)
% controller variables
persistent param yalmip_optimizer Th Dh Aaug Caug L Dh_csv

% Initialize param, MPC controller, Th, Dh, if not done already
if isempty(param)
    [param, yalmip_optimizer, Th, Dh, Aaug, Caug, L, Dh_csv] = init(Q,R,T,N);
end

% evaluate control action by solving MPC problem
% [u_mpc,errorcode] = yalmip_optimizer(...);
% if (errorcode ~= 0)
%     warning('MPC6 infeasible');
% end
% p = ...;

% observer update
% ...

% set point update
% ...

% Evaluate control action by solving MPC problem
[u_mpc,errorcode] = yalmip_optimizer(Th, Dh);
if (errorcode ~= 0)
    warning('MPC6 infeasible');
end
% p = u_mpc + param.p_sp;
p = u_mpc{1} + u_mpc{2};

% Observer update
aux = [Th; Dh];
aux = Aaug*aux + [param.B; zeros(3)]*p + L*(-T + Caug*aux);
Th = aux(1:3);
Dh = aux(4:6);
Dh_csv(:, size(Dh_csv,2)+1) = Dh;
writematrix(Dh_csv,'Dh.csv');

end

function [param, yalmip_optimizer, Th, Dh, Aaug, Caug, L, Dh_csv, p_sp] = init(Q,R,T,N)
% get basic controller parameters
% ...
% get terminal cost
% ...
% get terminal set
% ...
% design disturbance observer
% ...
% init state and disturbance estimate variables
% ...
% implement your MPC using Yalmip here
% nx = size(param.A,1);
% nu = size(param.B,2);
% U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
% X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
% T0 = sdpvar(nx,1,'full');
% T_sp = ...;
% p_sp = ...;
% constraints = [...];
% objective = ...;
% for k = 1:N-1
%     constraints = [constraints, ...];
%     objective = objective + ...;
% end
% constraints = [constraints, ...];
% objective = objective + ...;
% ops = sdpsettings('verbose',0,'solver','quadprog');
% yalmip_optimizer = optimizer(constraints,objective,ops,...,...);

% Get basic controller parameters
param = compute_controller_base_parameters;
yalmip('clear');
% Design disturbance observer
Aaug = [param.A, param.Bd; zeros(3), eye(3)];
Caug = [eye(3), zeros(3)];
% P = [0, 0, 0, 0.5, 0.5, 0.5];
P = [0, 0, 0, 0.5, 0.5, 0.5];
L = -(place(Aaug',Caug',P))';
% Init state and disturbance estimate variables
Th = T;
Dh = zeros(3, 1);
% Dh = [6000; -750; 400];
Dh_csv(:, 1) = Dh;

% Implement your MPC using Yalmip here
nx = size(param.A,2);
nu = size(param.B,2);
nd = size(param.Bd,2);
U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
error = sdpvar(repmat(2*nx,1,N),ones(1,N),'full');
T0 = sdpvar(nx,1,'full');
D0 = sdpvar(nd,1,'full');

% Get steady state target
r = [25; -42; -18.5];
sp = [eye(3)-param.A, -param.B; eye(3), zeros(3)] \ [param.Bd*D0; r];
T_sp = sp(1:3);
p_sp = sp(4:6);

% Set objective and constraints
X0 = T0 - T_sp;

objective = 0;
constraints = [];
constraints = [constraints, X{1} == X0];

for k = 1:N-1
    constraints = [constraints, X{k+1} == param.A * X{k} + param.B * U{k}];
    constraints = [constraints, param.Xcons(:,1) <= X{k+1} <= param.Xcons(:,2)];
    constraints = [constraints, param.Ucons(:,1) <= U{k} <= param.Ucons(:,2)];
    objective = objective + U{k}' * R * U{k} + X{k}' * Q * X{k};
end

[~, P_inf, ~] = dlqr(param.A, param.B, Q, R);
l_f = X{N}' * P_inf * X{N};
objective = objective + l_f;

[A_x, b_x] = compute_X_LQR(Q, R);
constraints = [constraints, A_x * X{N} <= b_x];
ops = sdpsettings('verbose', 0, 'solver', 'quadprog');
% yalmip_optimizer = optimizer(constraints, objective, ops, {T0,D0}, U{1});
yalmip_optimizer = optimizer(constraints, objective, ops, {T0,D0}, {U{1}, p_sp});

end