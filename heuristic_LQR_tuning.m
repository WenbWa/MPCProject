% BRRIEF:
%   Template for tuning of Q and R matrices for LQR controller as described 
%   in task 6.
% INPUT:
%   n_samples:  Number of samples considered for the tuning.
%   T0:         Initial condition
%   T_sp:       Set point
%   scen:       Disturbance scenario
% OUTPUT:
%   Q, R: Describes stage cost of LQR controller (x^T Q x + u^T R u)

function [Q, R] = heuristic_LQR_tuning(n_samples, T0, T_sp, scen)

% Initialize figure
figure; set(gcf, 'WindowStyle' ,'docked'); grid on; hold on
xlabel('Energy consumption [kWh]'); 
ylabel('Relative norm of steady state deviation');

% Parameter Initial
R = eye(3);
Q_best = eye(3);
power_best = Inf;
dT_best = Inf;
% Q Tuning for n_samples times
for index = 1:n_samples
    if mod(index, n_samples/10)==0
        fprintf('LQR Tuning Processed %d percent\n', 100*index/n_samples);
    end
    % Random Q Simulation
    Q_idx = [randi([1 10e6]), 0, 0;
             0, randi([1 10e6]), 0;
             0, 0, randi([1 10e6])];
    clear controller_lqr;
    [T, p, ~, ~, T_v, p_v] = simulate_building(T0, @controller_lqr, Q_idx, R, scen, 0);
    % Simulation Evaluation
    dT_relative = norm(T_sp - T(:, 15))/norm(T_sp - T0);
    power_sum = sum(abs(p), 'all')/1000/60;
    % Plot Evaluation
    if T_v==true
        scatter(power_sum, dT_relative, 50, 'red');
    elseif p_v==true
        scatter(power_sum, dT_relative, 50, 'blue');
    else
        scatter(power_sum, dT_relative, 50, 'green');
    end
    % Update Best Tuning Q
    if ~T_v
        if power_sum <= 16
            if dT_relative < dT_best
                power_best = power_sum;
                dT_best = dT_relative;
                Q_best = Q_idx;
                fprintf('Update: Best power_sum = %.3f, Best dT_relative = %.3f\n', power_sum, dT_relative);
            end
        end
    end
end

Q = Q_best;
scatter(power_best, dT_best, 100, 'black', 'filled');

end
