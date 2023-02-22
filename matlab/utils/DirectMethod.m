function [K] = DirectMethod(sys, gammaStar, x, u, method)
    % Direct method: data-driven synthesis from gathered training data
    if nargin < 5
        method = "lyapunov";
    end

    % Construct training data matrices ("snapshots")
    [~, X0, U0, X1, N] = get_snapshots(x, u);
    nx = sys.nx;
    nu = sys.nu;
    nz = nx + nu;

    D = sys.ctrl.D;
    H = sys.ctrl.H;
    G = sys.ctrl.G;

    if method == "lyapunov"
        % Solve for stabilizing feedback gains using (data-driven) LMIs
        cvx_clear; cvx_begin sdp quiet
            variables Q(N,nx) alph
            variable Y(nx,nx) symmetric
            maximize(alph)
            [...
                Y - alph*(X1*X1'), X1*Q;
                Q'*X1', Y;
            ] >= 0;
            [...
                eye(N), Q;
                Q', Y;
            ] >= 0;
            X0*Q == Y;
            alph >= 0;
        cvx_end;
        
        % Solve for state feedback gains
        K = (U0*Q)/Y;

    elseif method == "h2"
        % Solve for H2 optimal feedback gains using (data-driven) LMIs
        cvx_clear; cvx_begin sdp quiet
            variable Q(N,nx)
            variable X(nu,nu) symmetric
            variable Y(nx,nx) symmetric
            minimize (trace((H'*H)*X0*Q) + trace(X));
            subject to
            [...
                X, sqrt(G'*G)*U0*Q;
                (sqrt(G'*G)*U0*Q)', Y;
            ] >= 0;
            [...
                Y - eye(nx), X1*Q;
                Q'*X1', Y;
            ] >= 0;
            X0*Q == Y;
        cvx_end;

        % Solve for state feedback gains
        K = (U0*Q)/Y;
    elseif method == "h2s"
        % Solve for H2 suboptimal feedback gains using (data-driven) LMIs
        cvx_clear; cvx_begin sdp quiet
            variables Q(N,nx) slack
            variable X(nu,nu) symmetric
            variable Y(nx,nx) symmetric
            minimize (trace((H'*H)*X0*Q) + trace(X) + slack);
            subject to
            [...
                X, sqrt(G'*G)*U0*Q;
                (sqrt(G'*G)*U0*Q)', Y;
            ] >= 0;
            [...
                Y - eye(nx), X1*Q;
                Q'*X1', Y;
            ] >= 0;
            X0*Q == Y;
            slack <= 0;
        cvx_end;

        % Solve for state feedback gains
        K = (U0*Q)/Y;

    elseif method == "hinf"
        % Solve for H-infinity optimal feedback gains, using (data-driven) LMIs
        l_min = 100;
        l_max = 500;
        dl = 10;

        % Line search for feasible multiplier lambda
        for lambda = l_min:dl:l_max
            nz = nx + nu;
        
            % Quadratic constraint on disturbance set (norm-bounded by wbar)
            wbar = eps;%2e-2;
            Qw = -eye(nx);
            Sw = zeros(nx,N);
            Rw = wbar^2*eye(N);
        
            % Quadratic constraint on regulation performance (H-infinity norm <= gamma_)
            Qt = -gamDesired^2*eye(nx);
            St = zeros(nx,nz);
            Rt = eye(nz);

            % Solve for H-infinity optimal control
            tol = 1e-6;
            cvx_clear;
            cvx_begin sdp quiet
                variable Q(N,nx)
                variable Y(nx,nx) symmetric

                % Change of variables
                X0*Q == Y;

                % Data-driven H-infinity optimal control, Berberich et. al
                F1 = [...
                    -Y, (H*Y + G*U0*Q)'*St', -lambda*Q'*Sw';
                    St*(H*Y + G*U0*Q), Qt, zeros(nx,nx);
                    -lambda*Sw*Q, zeros(nx,nx), lambda*Qw;
                ];
                F2 = [... 
                    Q'*X1', (H*Y + G*U0*Q)', Q';
                    D', zeros(nx,nz), zeros(nx,N);
                    D', zeros(nx,nz), zeros(nx,N);
                ];
                F3 = [...
                    -Y, zeros(nx,nz), zeros(nx,N);
                    zeros(nz,nx), -inv(Rt), zeros(nz,N);
                    zeros(N,nx), zeros(N,nz), -inv(lambda*Rw);
                ];
                [...
                    F1, F2;
                    F2', F3;
                ] <= 0; %tol*eye(4*nx + nz + N);
            cvx_end
        
            fprintf('.');
            %lambda
            if strcmp('Innacurate/Solved', cvx_status)
                % Possible solution - compute H-infinity norm (for debugging)
                K = (U0*Q)/Y;
                %break;
            elseif strcmp('Solved', cvx_status)
                % Optimal solution - stop searching
                K = (U0*Q)/Y;
                break;
            end
        end
    end
end