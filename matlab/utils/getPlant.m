function [P_ol, T_ol, P_cl, T_mu, L_cl, T_K] = getPlant(A, B, D, H, G, F, dt, K)
    % Constructs various forms of the "full information" plant models:
    %
    % P_ol: open-loop standard/generic notation: the typical state-space
    %       plant, which encodes state dynamics and plant output equations.
    %
    %   x[k+1] = [ A | B ] x[k]
    %            [---|---]
    %     y[k] = [ C | 0 ] u[k]
    %
    %            |_______|
    %              P_ol
    %
    % T_ol: open-loop LFT notation: the "generalized plant" with regulator
    %       performance outputs z[k] as well as standard states and measurements.
    %
    %   x[k+1] = [ A | D  B ] x[k]
    %            [---|------]
    %     z[k] = [ H | F  G ] w[k]
    %     y[k] = [ C | 0  E ] u[k]
    %
    %            |__________|
    %                T_ol
    %
    % If a feedback gain matrix K (for feedback policy u[k] = mu(y[k]) = K y[k])
    % is provided, the function also constructs closed-loop plant models:
    %
    % P_cl: closed-loop standard/generic notation: P_ol with output feedback
    %       control policy u = K*y in place.  (Note: negative sign implied.)
    %
    %   x[k+1] = [ A + BKC | B ] x[k]
    %            [---------|---]
    %     y[k] = [ C       | 0 ] u[k]
    %
    %            |_____________|
    %                 P_cl
    %
    % T_mu: closed-loop LFT notation: T_ol with output feedback control
    %       policy u[k] = mu(y[k]) = K y[k] in place.  (Note: negative sign implied.)
    %
    %   x[k+1] = [ A + BKC | D + BKE ] x[k]
    %            [---------|---------]
    %     z[k] = [ H + GKC | F + GKE ] w[k]
    %
    %            |___________________|
    %                    T_mu
    %
    % L_cl: closed-loop loop transfer function, with output feedback control
    %       policy u[k] = mu(y[k]) = K y[k] in place.  (Note: negative sign implied.)
    %
    %            [ 0 | P_ol ]
    %   x[k+1] = [---|------] x[k]
    %            [ K |  0   ]
    %
    %            |__________|
    %                L_cl
    %
    % References:
    % [1] Zhou, K., & Doyle, J. C. (1998). Essentials of Robust Control
    %     (Vol. 104). Upper Saddle River, NJ: Prentice Hall.
    %
    % [2] Basar, T., Bernhard, P. (2008). H-infinity Optimal Control and
    %     Related Minimax Design Problems: A Dynamic Game Approach.
    %     Springer Science & Business Media.
    %

    nx = size(B, 1); % number of states
    nu = size(B, 2); % number of inputs
    E = 0*D; % weight of disturbance on outputs is zero for FI
    C = eye(nx); % output matrix for FI indicates full state observability

    % Open-loop standard/generic notation: the typical state-space plant,
    % which encodes state dynamics and plant output equations.
    P_ol = ss(A, B, C, 0, dt);

    % Open-loop LFT notation: the "generalized plant" with regulator
    % performance outputs z[k] as well as standard states and measurements.
    A_ol = A;
    B_ol = [D, B];
    C_ol = [H; C];
    D_ol = [F, G; zeros(nx,nu), E];
    T_ol = ss(A_ol, B_ol, C_ol, D_ol, dt);

    if nargin < 8
        % No feedback gains provided, can't calculate closed-loop systems
        P_cl = [];
        T_mu = [];
        L_cl = [];
    else
        % Closed-loop standard/generic notation: P_ol with output feedback
        % control policy u = K*y in place.  (Note: negative sign implied.)
        P_cl = ss(A + B*K, B, C, 0, dt);

        % Closed-loop LFT notation: T_ol with output feedback control
        % policy u = K*y in place.  (Note: negative sign implied.)
        A_cl = A + B*K*C;
        B_cl = D;
        C_cl = H + G*K*C;
        D_cl = F;
        T_mu = ss(A_cl, D, C_cl, F, dt);
        T_K = ss(A_cl, [D, 0*B], [C_cl; C], [F, G; 0*D, 0*B], dt);

        % Loop transfer matrix: akin to a transfer function, and square (so
        % it can be analyzed more easily in certain MATLAB functions).
        L_cl = [ss(zeros(nx)), P_ol; K, ss(zeros(nu))];
    end
end