function [testDir] = plot_baseline_SV(fignum, fudge)
    figure(fignum); clf; tl = tiledlayout(1,3,'TileSpacing','Compact');
    plot_system_SV(fignum, "double-integrator", fudge);
    plot_system_SV(fignum, "cart-pole", fudge);
    plot_system_SV(fignum, "robot-arm", fudge);
    ylabel(tl, 'Magnitude (dB)','FontName','Helvetica Neue','FontSize',16);
end

%% Subroutines
function [] = plot_system_SV(fignum, caseStudy, fudge)
    % Instantiate dynamical plant
    if caseStudy == "double-integrator"
        sys = DoubleIntegrator;
        tilenum = 1;
        titlestr = "Double Integrator";
    elseif caseStudy == "cart-pole"
        sys = CartPole;
        tilenum = 2;
        titlestr = "Cart Pole";
    elseif caseStudy == "robot-arm"
        sys = RobotArm;
        tilenum = 3;
        titlestr = "Robot Arm";
    end

    % Compute baseline state feedback controllers
    gamma_star = minimaxPolicy(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, eps, 1e5, fudge);
    [~, K_h2_b] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, Inf);
    [~, K_hinf_b] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, gamma_star);
    sys.ctrl.gamma = gamma_star;

    % Compute singular values and H-infinity norm of the generalized plant
    [P_ol, ~, ~, T_cl_h2_b] = sys.get_plant(K_h2_b);
    [P_ol, ~, ~, T_cl_hinf_b] = sys.get_plant(K_hinf_b);
    SV_h2_b = sigma(T_cl_h2_b, sys.W);
    SV_hinf_b = sigma(T_cl_hinf_b, sys.W);

    % Compute disk-based stability margins
    MMIO = diskmargin(P_ol, -1*K_h2_b);
    DiskMargin_h2_b = 2*MMIO.DiskMargin % loop margin w/ simultaneous I/O variation
    MMIO = diskmargin(P_ol, -1*K_hinf_b);
    DiskMargin_hinf_b = 2*MMIO.DiskMargin % loop margin w/ simultaneous I/O variation

    figure(fignum); nexttile(tilenum);
    semilogx(sys.W, mag2db(gamma_star*(fudge/gamma_star))*ones(1,length(sys.W)), 'k--',...
        sys.W, mag2db(SV_h2_b(1,:)*(fudge/gamma_star)), 'r',...
        sys.W, mag2db(SV_hinf_b(1,:)*(fudge/gamma_star)), 'k');
    title(titlestr,'FontName','Helvetica Neue','FontSize',16);
    set(gca,'XScale','log'); xlim([1e-3 pi/sys.dt]); grid on;
    ylim([-35 11]);
    legend('$\gamma$', '$\bar{\sigma}$ ($\mathcal{H}_{2}$)',...
        '$\bar{\sigma}$ ($\mathcal{H}_{\infty}$)',...
        'Interpreter','latex', 'FontSize',14,'Location','SouthWest');
    xlabel('Frequency (rad/s)','FontName','Helvetica Neue','FontSize',16);
end