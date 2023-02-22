function [W0, X0, U0, X1, N] = get_snapshots(x, u, w)
    %% Construct data matrices ("snapshots")
    N = length(x);
    X0 = x(:, 1:N-1);
    X1 = x(:, 2:N);
    U0 = u(:, 1:N-1);
    if nargin < 3
        W0 = 0*X0;
    else
        W0 = w(:, 1:N-1);
    end
    N = length(X0);
    nx = size(X0, 1);
    nu = size(U0, 1);

    Assumption1 = (rank([X0; U0]) == nx + nu) && (rank(X1) == nx);
    if ~Assumption1
        warning('Persistent excitation assumption possibly invalid');
    end
end