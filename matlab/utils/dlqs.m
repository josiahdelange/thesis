function [sysd_cl, K, Ki] = dlqs(Ad, Bd, Q, R, Ci, Qi, dt)
    % Get dimensions of matrices
    nx = size(Bd, 1); % number of feedback states
    nu = size(Bd, 2); % number of control inputs
    nt = size(Ci, 1); % number of tracked/integral states

    % Construct the augmented open-loop system matrices
    Ad_ = [Ad, zeros(nx, nt); Ci, eye(nt)];
    Bd_ = [Bd; zeros(nt, nu)];

    % Check if system is controllable
    n_uc = (nx + nt) - rank(ctrb(Ad_, Bd_), eps);
    if n_uc
        warning(['LQ servo system has ' num2str(n_uc) ' uncontrollable modes!']);
    end

    % Construct the augmented state cost matrix (note: input cost unchanged)
    Q_ = diag([diag(Q); diag(Qi)]);

    % Solve for LQR feedforward gains
    [Kff, ~] = dlqr(Ad, Bd, Q, R);

    % Compute servo (= PD and I) gains using discrete-time, infinite-horizon LQR
    [Kservo, ~] = dlqr(Ad_, Bd_, Q_, R);

    % Decompose into PD and integral gains
    K = Kservo(1:nu, 1:nx); % feedback gains
    Ki = Kservo(1:nu, nx+(1:nt)); % integral gains

    % Construct the augmented closed-loop system matrices, with LQ servo
    Ad_cl = [Ad - Bd*K, -Bd*Ki; Ci, eye(nt, nt)];
    Bd_cl = [Bd*Kff; -Ci];
    Cd_cl = [eye(nx), zeros(nx, nt)];

    % Discrete-time closed loop system
    sysd_cl = ss(Ad_cl, Bd_cl, Cd_cl, 0, dt);
end
