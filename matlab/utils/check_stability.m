function [stable] = check_stability(A, B, K)
    stable = ~any(isnan(K(:))) && ~any(abs(eig(A + B*K)) >= 1);
end