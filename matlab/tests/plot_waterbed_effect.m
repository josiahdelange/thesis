function [] = plot_waterbed_effect(fignum, fudgeSweep)
    % Run test_uncertainty22 for all three case studies
    figure(fignum); clf; tl = tiledlayout(4,3,'TileSpacing','Compact');
    %  1  2  3
    %  4  5  6
    %  7  8  9
    % 10 11 12
    plot_waterbed_single(fignum, "double-integrator", fudgeSweep);
    plot_waterbed_single(fignum, "cart-pole", fudgeSweep);
    plot_waterbed_single(fignum, "robot-arm", fudgeSweep);
end

function [] = plot_waterbed_single(fignum, caseStudy, fudgeSweep)
    if nargin < 3
        error('Must specify fudgeSweep');
    end
    if caseStudy == "double-integrator"
        sys = DoubleIntegrator;
        svColOffset = 0;
        capitalizedName = "Double Integrator";
    elseif caseStudy == "cart-pole"
        sys = CartPole;
        svColOffset = 1;
        capitalizedName = "Cart Pole";
    elseif caseStudy == "robot-arm"
        sys = RobotArm;
        svColOffset = 2;
        capitalizedName = "Robot Arm";
    end

    % Generalized open-loop plant
    [~, T_ol, ~, T_K_laub] = sys.get_plant;

    % Compute the minimax disturbance attenuation gain (gamma)
    gamma_star = minimaxPolicy(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, eps, 1e5);
    disp(['Bisection Gamma: ' num2str(mag2db(gamma_star)) ' dB']);

    %% Feedback control synthesis (MATLAB Robust Control Toolbox)
    [~,~,~,info] = hinfsyn(T_ol, sys.nx, sys.nu);
    gamma_matlab = info.gamma;
    disp(['MATLAB Gamma: ' num2str(mag2db(gamma_matlab)) ' dB']);
    K_toolbox = info.Ku;
    [~, ~, ~, T_K_toolbox] = sys.get_plant(K_toolbox);
    [SV_matlab, ~] = sigma(T_K_toolbox, sys.W);
    SV_matlab = SV_matlab/gamma_star;

    %% Feedback control synthesis (custom DGARE solver + gamma bisection)
    SV_max = zeros(length(fudgeSweep), size(sys.W, 2));
    seq = [1, 4, 7, 10];
    for ii = 1:length(fudgeSweep)
        % Obtain fudge factor on optimal/desired/target gamma
        fudge = fudgeSweep(ii);

        % Compute H-infinity controller by solving the discrete game ARE
        gamma_ = fudge*gamma_star;
        [~, K_iter] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, gamma_, "iterative");

        % Compute singular values and H-infinity norm of the generalized plant
        [~, ~, ~, T_K_iter] = sys.get_plant(K_iter);
        [SV_iter, ~] = sigma(T_K_iter, sys.W);
        SV_iter = SV_iter/gamma_star;

        % Plot result, for this fudge factor
        figure(fignum); nexttile(seq(ii) + svColOffset); hold on;
        semilogx(sys.W, mag2db(gamma_/gamma_star)*ones(1,length(sys.W)), 'k--');
        semilogx(sys.W, mag2db(SV_matlab(1,:)), 'b');
        semilogx(sys.W, mag2db(SV_iter(1,:)), 'k', 'LineWidth', 0.9);
        figure(fignum); nexttile(seq(ii) + svColOffset); hold off; grid on;
        if fudge == 1
            gamLegendString = ['$\gamma = \gamma^{*}$'];
        else
            gamLegendString = ['$\gamma = ' num2str(fudge) '\gamma^{*}$'];
        end
        legend(gamLegendString, '$\bar{\sigma}$ (Toolbox)',...
            '$\bar{\sigma}$ (Iterative)', 'Interpreter','latex',...
            'FontSize',14,'Location','SouthWest');
        set(gca,'XScale','log'); xlim([1e-3; pi/sys.dt]); yl = ylim;
        if yl(2) < 0.7
            yl(2) = 0.7; ylim(yl);
        end

        if ii == length(fudgeSweep)
            ylim([-30 5]);
        end
    end
    xlabel('Frequency (rad/s)', 'FontName','Helvetica Neue','FontSize',16);
    figure(fignum); nexttile(1 + svColOffset);
    title(capitalizedName, 'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
    for t_ii_1 = [1, 4, 7, 10]
        figure(fignum); nexttile(t_ii_1);
        ylabel('Magnitude (dB)','FontName','Helvetica Neue','FontSize',16);
    end    
end