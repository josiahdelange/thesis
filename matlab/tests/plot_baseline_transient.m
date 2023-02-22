function [] = plot_baseline_transient(fignum, caseStudy)
    figure(fignum); clf; plot_system_tr(fignum, caseStudy);
end

%% Subroutines
function [] = plot_system_tr(fignum, caseStudy)
    % Instantiate dynamical plant
    if caseStudy == "double-integrator"
        sys = DoubleIntegrator;
        titlestr = "Double Integrator";
        tl = tiledlayout(3,1,'TileSpacing','Compact');
    elseif caseStudy == "cart-pole"
        sys = CartPole;
        titlestr = "Cart Pole";
        tl = tiledlayout(3,2,'TileSpacing','Compact');
    elseif caseStudy == "robot-arm"
        sys = RobotArm;
        titlestr = "Robot Arm";
        tl = tiledlayout(3,2,'TileSpacing','Compact');
    end

    % Compute baseline state feedback controllers
    fudge = 1.5;
    gamma_star = minimaxPolicy(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, eps, 1e5, fudge);
    [~, K_h2_b] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, Inf);
    [~, K_hinf_b] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, gamma_star);
    sys.ctrl.gamma = gamma_star;

    % Gather transient response data
    disp('  Testing baseline H2');
    [stable, ~, ~, ~, ~, ~, ~, ~, ~, t, x, u, y, w, SV] = sys.run_test(K_h2_b);
    if stable
        sys.add_plot_transient_response(fignum, t, u, y, 'r', false);
    end
    disp('  Testing baseline Hinf');
    [stable, ~, ~, ~, ~, ~, ~, ~, ~, t, x, u, y, w, SV] = sys.run_test(K_hinf_b);
    if stable
        sys.add_plot_transient_response(fignum, t, u, y, 'k', false);
    end

    title(tl,titlestr,'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
end