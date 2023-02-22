function [K] = IndirectMethod(sys, gammaStar, x, u, method)
    % Indirect method: model-based synthesis using plant identified from training data
    if nargin < 5
        method = "dmdc";
    end

    % Construct training data matrices ("snapshots")
    [~, X0, U0, X1, N] = get_snapshots(x, u);
    nx = sys.nx;
    nu = sys.nu;

    if method == "dmdc"
        % Solve total least squares problem via singular value decomposition
        [U_, S_, V_] = svd([X0; U0], 'econ');
        U_ = U_(:,1:(nx + nu));
        S_ = S_(1:(nx + nu),1:(nx + nu));
        V_ = V_(:,1:(nx + nu));
    
        % Estimate discretized state and input transition matrices
        G_hat = X1*V_*inv(S_)*U_';
        A_hat = G_hat(1:nx,1:nx);
        B_hat = G_hat(:,nx+1:end);

    elseif method == "cvx"
        % Solve total least squares problem via convex optimization
        cvx_clear; cvx_begin sdp quiet
            variable G_hat(nx,nx+nu)
            minimize(norm(G_hat*[X0; U0] - X1, 'fro'))
        cvx_end

        % Estimate discretized state and input transition matrices
        A_hat = G_hat(1:nx,1:nx);
        B_hat = G_hat(:,nx+1:end);
    else
        error('Invalid method: dmdc or cvx');
    end

    % Solve for control gains using estimated model matrices
    [~, K] = solveDGARE(A_hat, B_hat, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, gammaStar);
end