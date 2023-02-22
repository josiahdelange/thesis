function [gamma_, P, K] = minimaxPolicy(A, B, D, H, G, initLB, initUB, fudge)
    % Solves the minimax H-infinity optimal control problem, searching for
    % the lowest achievable gamma (disturbance attenuation gain) of
    % infinite-horizon soft-constrained cost functional from [2]:
    %
    %   L_gamma(u, w) = 1/N * (sum {x'Qx + u'Ru - gamma^2 w'w}
    %                 = |z|^2 - gamma^2 |w|^2
    %                 = ||z|| - gamma^2 ||w||
    %
    % The optimal minimum cost (||z||) maximum disturbance (||w||), i.e.,
    % "minimax" saddle point of the dynamic game occurs for a particular
    % value of disturbance attenuation gain gamma, which then directly
    % corresponds to the H-infinity norm.
    %
    % The H-infinity optimal control policy is found by searching (via
    % bisection) within the range [initLB, initUB] for the minimax gamma,
    % and solving the discrete game algebraic Riccati equation (DGARE) at
    % this value, which provides the feedback gains K.  The closed-loop
    % "generalized plant" (see also getPlant.m):
    %
    %    T_mu(z) = F + (H + GK)(zI - (A + BK))^-1 D
    %
    % then has an H-infinity norm which is < gamma
    %
    %    ||T_mu(z)||_{Inf} <= gamma
    %
    % Implementation notes:
    %   - A "fudge factor" of 1.1 is added to the end of the search,
    %     meaning the value of gamma used for control synthesis is
    %     technically a LITTLE more conservative than the "minimax" value,
    %     which has been shown to increase robustness (e.g. disk margins).
    %     If desired, this could be adjusted, provided it is >= 1.0.
    %
    %   - Both techniques assume the "square root" form of the LQ cost
    %     matrices H = Qx'*Qx and G = Ru'*Ru, such that the factorization
    %     [Qx 0; 0 Ru] = [H; G]*[H', G'] holds.
    %
    %   - If gamma_ = Inf (or D = 0), this function computes/solves the
    %     discrete algebraic Riccati equation (DARE) for standard LQR.
    %
    % References:
    % [1] Laub, A. (1979). A Schur Method for Solving Algebraic Riccati
    %     Equations. IEEE Transactions on Automatic Control, 24(6), 913-921.
    %
    % [2] Basar, T., Bernhard, P. (2008). H-infinity Optimal Control and
    %     Related Minimax Design Problems: A Dynamic Game Approach.
    %     Springer Science & Business Media.
    %
    if nargin < 8
        fudge = 1.0;
    end
    
    % Bisection method for best (min-max) gamma
    LB = initLB;
    UB = initUB;
    tol = 1e-9;
    maxIter = 100;
    for ii = 1:maxIter
        % Check for convergence
        if (UB - LB) < tol
            % Converged, gamma_ currently = optimal
            break;
        else
            % Keep searching, update gamma_ via bisection
            gamma_ = (LB + UB)/2;
        end

        % H-infinity synthesis, evaluation of new policy
        [P, K] = solveDGARE(A, B, D, H, G, gamma_, "iterative");

        % Spectral radius condition (6.7 or equivalently 3.7 of [2])
        if max(abs(eig(P*(D*D')))) >= gamma_^2
            % DGARE solution isn't positive definite
            LB = gamma_;
            continue;
        end

        % Closed-loop stability check
        if max(abs(eig(A + B*K))) >= 1
            % Feedback system is unstable
            LB = gamma_;
            continue;
        end

        % If this point is reached, gamma was feasible; use a bisection
        % (on the next loop iteration) to compute a new value
        UB = gamma_;
    end

    % Solve the DGARE for optimal value of gamma_
    gamma_ = fudge*gamma_; % reduces sensitivity of minimax solution
    [P, K] = solveDGARE(A, B, D, H, G, gamma_, "iterative");
end