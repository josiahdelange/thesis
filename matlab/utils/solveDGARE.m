function [P, K] = solveDGARE(A, B, D, H, G, gamma_, method)
    % Solves the discrete generalized algebraic Riccati equation (DGARE),
    % using either non-iterative Schur decomposition of the symplectic
    % (discrete-time Hamiltonian) matrix [1], or a direct iterative
    % approach on the discrete Riccati equation [2].
    %
    % Both techniques assume the "square root" form of the LQ cost matrices
    % H = Qx'*Qx and G = Ru'*Ru, such that [Qx 0; 0 Ru] = [H; G]*[H', G'].
    %
    % If gamma_ = Inf (or D = 0), this function computes/returns the solution
    % to the discrete algebraic Riccati equation (DARE) for LQR.
    %
    % References:
    % [1] Laub, A. (1979). A Schur Method for Solving Algebraic Riccati
    %     Equations. IEEE Transactions on Automatic Control, 24(6), 913-921.
    %
    % [2] Basar, T., Bernhard, P. (2008). H-infinity Optimal Control and
    %     Related Minimax Design Problems: A Dynamic Game Approach.
    %     Springer Science & Business Media.
    %

    nx = size(B, 1);
    nu = size(B, 2);
    if nargin < 7
        method = "iterative";
    end

    % LQ state weighting matrix
    Q = H'*H; % LQ state weighting matrix
    Nxu = zeros(nx,nu); % LQ state-input cross-weighting matrix (zero-ed)

    % Scale the plant matrices according to factorization
    % [Qx, Nxu; Nxu', Ru] = [H; G]*[H', G']
    B_ = [B, zeros(nx,nx+nu)];
    N_ = [Nxu, H'];
    R_ = [zeros(nu), G'; G, -eye(nx+nu)];

    if method == "vaughan"
        % Compute the sympletic matrix for DGARE
        Psi = [...
            inv(A), A\(B_/R_*B_' - gamma_^-2*(D*D'));
            Q/A, A' + Q*A\(B_/R_*B_' - gamma_^-2*(D*D'));
        ];

        % Compute eigendecomposition of Psi
        [E_, Lambda_] = eig(Psi);

        % Compute solution to DGARE
        E11 = E_(1:nx,1:nx);
        E21 = E_(nx+1:end,1:nx);
        P = real(E21/E11);
        P = 0.5*(P + P'); % enforce symmetry

        % Compute state feedback policy
        Lambda = (inv(P) + B_/R_*B_' - gamma_^-2*(D*D'));
        Lambda = (Lambda + Lambda')/2; % enforce symmetry
        K = -R_\(B_'/Lambda*A + N_');
        K = real(K(1:min(nu,end),:));

    elseif method == "laub"
        % Compute the sympletic matrix for DGARE
        Psi = [...
            inv(A), A\(B_/R_*B_' - gamma_^-2*(D*D'));
            Q/A, A' + Q*A\(B_/R_*B_' - gamma_^-2*(D*D'));
        ];
    
        % Compute Schur decomposition of Psi
        [U_, T_] = schur(Psi, 'real');
        [U_, ~] = ordschur(U_, T_, 'udo');
        U11 = U_(1:nx, 1:nx);
        U21 = U_(nx+1:end, 1:nx);
  
        % Compute solution to DGARE
        P = U21/U11;
        P = 0.5*(P + P'); % enforce symmetry

        % Compute state feedback policy
        Lambda = (inv(P) + B_/R_*B_' - gamma_^-2*(D*D'));
        Lambda = (Lambda + Lambda')/2; % enforce symmetry
        K = -R_\(B_'/Lambda*A + N_');
        K = real(K(1:min(nu,end),:));

    elseif method == "iterative"
        tol = 1e-9;
        maxIter = 1e3;
        P = Q;
        for ii = 1:maxIter
            % Store previous iteration
            P_old = P;
        
            % Compute next iteration of ARE (asymmetric version)
            Lambda = (inv(P_old) + B_/R_*B_' - gamma_^-2*(D*D'));
            Lambda = (Lambda + Lambda')/2; % enforce symmetry
            P = Q + A'/Lambda*A;
            P = (P + P')/2; % enforce symmetry

            % Compute state feedback policy
            K = -R_\(B_'/Lambda*A + N_');
            K = real(K(1:min(nu,end),:));

            % Spectral radius condition (6.7 or equivalently 3.7 of [2])
            if max(abs(eig(P*(D*D')))) >= gamma_^2
                % P is not PD
                break;
            end

            % Check for convergence
            if norm(P - P_old) <= tol
                break;
            end
        end

    elseif method == "h2syn"
        % Obtain the generalized plant model
        T_ol = getPlant(A, B, D, H, G, F, dt);

        % Solve Riccati equation for state feedback gains
        [~, ~, ~, info] = h2syn(T_ol, nx, nu);
        P = info.X;
        K = info.Ku;

    elseif method == "h2lmi"
        % Solve LMI given in 4.2.2 of [1], for discrete-time H2-optimal
        % full-state feedback control
        %
        % [1] Caverly, R. J., & Forbes, J. R. (2019). LMI properties and
        % applications in systems, stability, and control theory. arXiv
        % preprint arXiv:1903.08599.
        cvx_clear; cvx_begin sdp quiet
            variable P(nx,nx) symmetric
            variable Z(nx+nu,nx+nu)
            variable Fd(nu,nx)
            variable tol
            minimize(tol)
            subject to
            P >= 0;
            Z >= 0;
            [...
                P, A*P + B*Fd, D;
                P'*A' + Fd'*B', P, zeros(nx,nx);
                D', zeros(nx,nx), eye(nx);
            ] >= 0;
            [...
                Z, H*P + G*Fd;
                P'*H' + Fd'*G', P;
            ] >= 0;
            trace(Z) <= tol;
        cvx_end;

        % Compute state feedback gains
        K = Fd/P;

    elseif method == "hinfsyn"
        % Obtain the generalized plant model
        T_ol = getPlant(A, B, D, H, G, zeros(nx + nu, size(D,2)), dt);

        % Solve Riccati equation for state feedback gains
        [~, ~, ~, info] = hinfsyn(T_ol, nx, nu, gamma_);
        P = info.X;
        K = info.Ku;

    elseif method == "hinflmi"
        % Solve LMI given in 4.3.2 of [1], for discrete-time H-infinity
        % optimal full-state feedback control
        %
        % [1] Caverly, R. J., & Forbes, J. R. (2019). LMI properties and
        % applications in systems, stability, and control theory. arXiv
        % preprint arXiv:1903.08599.
        cvx_clear; cvx_begin sdp quiet
            variable P(nx,nx) symmetric
            variable Fd(nu,nx)
            P >= 0;
            [...
                P,                A*P + B*Fd,        D,               zeros(nx,nx+nu);
                (A*P + B*Fd)',    P,                 zeros(nx,nx),    P*H' + Fd'*G';
                D',               zeros(nx,nx),      gamma_*eye(nx),  F';
                zeros(nx+nu,nx),  (P*H' + Fd'*G')',  F,               gamma_*eye(nx+nu);
            ] >= 0;
        cvx_end;

        % Compute state feedback gains
        K = Fd/P;
    end
end
