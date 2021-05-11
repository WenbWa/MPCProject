% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% INPUT:
%   Q, R: State and input weighting matrix, dimension (3,3)
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}

function [A_x, b_x] = compute_X_LQR(Q, R)
    % Get basic controller parameters
    persistent param;
    if isempty(param)
        param = compute_controller_base_parameters;
    end
    
    % Implement the X_LQR computation and assign the result
    % Computes a control Invariant set for LQR system x^+ = A*x+B*K*x
    K = -dlqr(param.A, param.B, Q, R);
    systemLQR = LTISystem('A', param.A+param.B*K);
    % Get State Constraint Set
    Xp = Polyhedron('A', [eye(3); -eye(3); K; -K], 'b', [param.Xcons(:,2); -param.Xcons(:,1); param.Ucons(:,2); -param.Ucons(:,1)]);
    
    % Get Control Invariant Set
    systemLQR.x.with('setConstraint');
    systemLQR.x.setConstraint = Xp;
    InvSetLQR = systemLQR.invariantSet();
    
    
%     subplot(2,1,1)
%     plot3(-2.25, 1.75, 0.75, '-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
% %     plot3(1.5, 2.75, -0.25, '-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
%     hold on
%     Xp.plot(), alpha(0.25), title('State Constraint Set under LQR Control'), xlabel('x_1'), ylabel('x_2');
%     subplot(2,1,2)
%     figure;
% %     plot3(-2.25, 1.75, 0.75, '-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
%     plot3(1.5, 2.75, -0.25, '-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
%     hold on
%     InvSetLQR.plot(), alpha(0.25), title('Control Invariant Set under LQR Control'), xlabel('x_1'), ylabel('x_2')
    
    A_x = InvSetLQR.A;
    b_x = InvSetLQR.b;
end