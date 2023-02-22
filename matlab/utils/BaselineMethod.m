function [K] = BaselineMethod(sys, gammaStar)
    % Baseline method: model-based synthesis from known plant
    [~, K] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, gammaStar);
end