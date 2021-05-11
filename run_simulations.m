%% Init
clear all
close all
addpath(genpath(cd));
rng(1234)
load('system/parameters_scenarios.mat')

param = compute_controller_base_parameters;
T_sp = param.T_sp;

% dT0_exmaple = ...
% T0_example = ...


%% Example
% figure; set(gcf, 'WindowStyle' ,'docked');
% clear persisten variables of function controller_example
% clear controller_example
% execute simulation
% [T,~,~,t] = simulate_building(T0_example,@controller_example);


%% Autonomous system simulation
disp('Autonomous system simulation started');

% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T_sp);

disp('Autonomous system simulation finished');


%% Heuristic LQR tuning
disp('Heuristic LQR tuning started');
% Tuning of LQR on first initial condition
T01 = T_sp + [-2.25; 1.75; 0.75];
T02 = T_sp + [1.5; 2.75; -0.25];

% [Q1, R1] = heuristic_LQR_tuning(2500, T01, T_sp, scen1)
% disp('Heuristic LQR tuning for T01 finished');
% 
% [Q2, R2] = heuristic_LQR_tuning(2500, T02, T_sp, scen1)
% disp('Heuristic LQR tuning for T02 finished');

% Tuning With input constraints
% Q1 = diag([4109479, 2064982, 2076632]);
% R1 = eye(3);
% Tuning Without input constraints
Q1 = diag([4973679, 5427908, 4949349]);
R1 = eye(3);

% Tuning With input constraints
% Q2 = diag([3618351, 912436, 452536]);
% R2 = eye(3);
% Tuning Without input constraints
Q2 = diag([5836090, 1342679, 3555303]);
R2 = eye(3);

disp('Heuristic LQR tuning finished');


%% Unconstrained LQR Optimal Control
disp('Unconstraint LQR optimal control started');

% clear controller_lqr;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T01, @controller_lqr, Q1, R1, scen1);
% disp('Unconstraint LQR optimal control for T01 finished');
% 
% clear controller_lqr;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T02, @controller_lqr, Q2, R2, scen1);
% disp('Unconstraint LQR optimal control for T02 finished');

disp('Unconstraint LQR optimal control finished');

%% Control Invariant Set XLQR
% disp('Compute Control Invariant Set X_LQR started');

% compute_X_LQR(Q2, R2);

% disp('Compute Control Invariant Set X_LQR finished');


%% From LQR to MPC
disp('From LQR to MPC started'); 
 
% clear controller_lqr;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T01, @controller_lqr, Q1, R1, scen1);
% clear controller_lqr;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T02, @controller_lqr, Q2, R2, scen1);
% 
% clear controller_mpc_1;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T01, @controller_mpc_1, Q1, R1, scen1);
% clear controller_mpc_1;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T02, @controller_mpc_1, Q2, R2, scen1);

% clear controller_mpc_2;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T01, @controller_mpc_2, Q1, R1, scen1);
% clear controller_mpc_2;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T02, @controller_mpc_2, Q2, R2, scen1);
% 
% clear controller_mpc_3;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T01, @controller_mpc_3, Q1, R1, scen1);
% clear controller_mpc_3;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T02, @controller_mpc_3, Q2, R2, scen1);

disp('From LQR to MPC finished');



%% Soft-constrained MPC
disp('Soft Constrained MPC started');

% clear controller_mpc_3;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T01, @controller_mpc_3, Q1, R1, scen2);
% clear controller_mpc_4;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T01, @controller_mpc_4, Q1, R1, scen2);
% 
% clear controller_mpc_3;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T01, @controller_mpc_3, Q1, R1, scen1);
% clear controller_mpc_4;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T01, @controller_mpc_4, Q1, R1, scen1);


% N = 30;
% % Using recorded disturbances
% d = [scen2.d_VC_scen; scen2.d_F1_scen;scen2.d_F2_scen];
% d = [d, zeros(3, N)];
% % Using smoothed disturbances
% d = smoothdata(d, 2, 'movmedian', 5);
%     
% clear controller_mpc_5;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T01, @controller_mpc_5, Q1, R1, scen2, 1, N, d);
% 
% figure;
% subplot(3,1,1)
% plot(scen2.d_VC_scen, '-o','Color','b')
% title('scen2.dVC'), xlabel('t'), ylabel('d');
% subplot(3,1,2)
% plot(scen2.d_F1_scen, '-o','Color','b')
% title('scen2.dF1'), xlabel('t'), ylabel('d');
% subplot(3,1,3)
% plot(scen2.d_F2_scen, '-o','Color','b')
% title('scen2.dF2'), xlabel('t'), ylabel('d');
% 
% figure;
% subplot(3,1,1)
% plot(d(1, :), '-o','Color','b')
% title('scen2.dVC smoothed'), xlabel('t'), ylabel('d');
% subplot(3,1,2)
% plot(d(2, :), '-o','Color','b')
% title('scen2.dF1 smoothed'), xlabel('t'), ylabel('d');
% subplot(3,1,3)
% plot(d(3, :), '-o','Color','b')
% title('scen2.dF2 smoothed'), xlabel('t'), ylabel('d');

disp('Soft Constrained MPC finished');


%% Offset-free MPC
disp('Offset-free MPC started');

% clear controller_mpc_3;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T01, @controller_mpc_3, Q1, R1, scen3);
% 
% clear controller_mpc_6;
% figure; set(gcf, 'WindowStyle' ,'docked');
% [T,p,~,~,T_v,p_v] = simulate_building(T01, @controller_mpc_6, Q1, R1, scen3);
% 
% 
% Dh = readmatrix('Dh.csv');
% 
% figure;
% subplot(3,1,1)
% plot(scen3.d_VC_scen+scen3.a_Env_VC * scen3.T_Env, '-o','Color','b')
% hold on
% plot(Dh(1, :), '-o','Color','r')
% title('scen3.dVC + scen3.a_Env_VC * scen3.T_Env'), xlabel('t'), ylabel('d');
% subplot(3,1,2)
% plot(scen3.d_F1_scen, '-o','Color','b')
% hold on
% plot(Dh(2, :), '-o','Color','r')
% title('scen3.dF1'), xlabel('t'), ylabel('d');
% subplot(3,1,3)
% plot(scen3.d_F2_scen, '-o','Color','b')
% hold on
% plot(Dh(3, :), '-o','Color','r')
% title('scen3.dF2'), xlabel('t'), ylabel('d');

disp('Offset-free MPC finished');

%% Comparison using forces
disp('MPC Implementation with FORCES Pro started');

clear controller_mpc_1;
figure; set(gcf, 'WindowStyle' ,'docked');
[T,p,~,t_sim,T_v,p_v] = simulate_building(T02, @controller_mpc_1, Q2, R2, scen1);

clear controller_mpc_1_forces;
figure; set(gcf, 'WindowStyle' ,'docked');
controller_mpc_1_forces(Q2, R2, T02, 30);
[T,p,~,t_sim_forces,T_v,p_v] = simulate_building(T02, @controller_mpc_1_forces, Q2, R2, scen1);


disp('MPC Implementation with FORCES Pro finished');
