function [] = run_all_singular_values(fignums)
    %% Runs all the uncertainty and noise tests to get singular values
    outputDir = '/Users/josiah/ms-thesis/latex/figures';

    %% Canonical/optimal LMI formulation for H2 direct
    tic;

    % Singular values: parameter uncertainty
    disp('--> Running scaled uncertainty test');
    run_uncertainty_tests(fignums(1), 100, false);
    fix_ylimits(fignums(1), [-35 11], [-35 11], [-35 11])

    % Singular values: measurement noise
    disp('--> Running scaled noise test');
    run_noise_tests(fignums(2), 100, false);
    fix_ylimits(fignums(2), [-35 20], [-35 20], [-35 20])

    toc;

    %% Adjusted/suboptimal LMI formulation for H2 direct
    tic;

    % Singular values: parameter uncertainty
    disp('--> Running scaled uncertainty test (suboptimal)');
    run_uncertainty_tests(fignums(3), 100, true);
    fix_ylimits(fignums(3), [-35 25], [-35 25], [-35 25])

    % Singular values: measurement noise
    disp('--> Running scaled noise test (suboptimal)');
    run_noise_tests(fignums(4), 100, true);
    fix_ylimits(fignums(4), [-35 30], [-35 30], [-35 30])

    toc;

    %% Save output files to disk
    % Save .fig files
    disp('Saving .fig files...');
    savefig(fignums(1), fullfile(outputDir, 'uncertainty_singular_values3'));
    savefig(fignums(2), fullfile(outputDir, 'noise_singular_values3'));
    savefig(fignums(3), fullfile(outputDir, 'uncertainty_singular_values4_s'));
    savefig(fignums(4), fullfile(outputDir, 'noise_singular_values4_s'));
    disp('...done.');
    
    % Save .png files
    disp('Saving .png files...');
    saveas(fignums(1), fullfile(outputDir, 'uncertainty_singular_values3.png'));
    saveas(fignums(2), fullfile(outputDir, 'noise_singular_values3.png'));
    saveas(fignums(3), fullfile(outputDir, 'uncertainty_singular_values4_s.png'));
    saveas(fignums(4), fullfile(outputDir, 'noise_singular_values4_s.png'));
    disp('...done');
end

%% Subroutines
function [lg, lh1, lh2, lh3] = test_and_plot_uncertainty(fignum, tilenum, caseStudy,...
    numRuns, scaleFactor, errThreshDB, suboptimal)

    % Testing data-driven control with inertial uncertainty
    if caseStudy == "double-integrator"
        sys = DoubleIntegrator;
        capitalizedName = "Double Integrator";
    elseif caseStudy == "cart-pole"
        sys = CartPole;
        capitalizedName = "Cart Pole";
    elseif caseStudy == "robot-arm"
        sys = RobotArm;
        capitalizedName = "Robot Arm";
    end

    % Compute baseline state feedback controllers
    gammaDesign = minimaxPolicy(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, eps, 1e5, 1.5);
    [~, K_h2_b] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, Inf);
    [~, K_hinf_b] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, gammaDesign);
    sys.ctrl.gamma = gammaDesign;

    figure(fignum); nexttile(tilenum); cla; hold on;
    lg = semilogx(sys.W,...
        mag2db(gammaDesign*(1.5/gammaDesign))*ones(1,length(sys.W)), 'k--',...
        'LineWidth', 1);
    grid on; xlim([1e-3; pi/sys.dt]); 

    SVmax_h2_i = nan(numRuns, size(sys.W, 2));
    SVmax_h2_d = nan(numRuns, size(sys.W, 2));
    SVmax_hinf_i = nan(numRuns, size(sys.W, 2));

    DM_h2_i = zeros(numRuns, 1);
    DM_h2_d = zeros(numRuns, 1);
    DM_hinf_i = zeros(numRuns, 1);

    rng(1); % initialize the psuedo-random number generator
    for n = 1:numRuns
        disp([' Running test case ' num2str(n) ' of ' num2str(numRuns)]);

        % Scale the variance of zero-mean, single-parameter uncertainty
        sys = sys.parameter_uncertainty(scaleFactor);

        % Gather training data from the full system, check feasiblity conditions
        [t_t, x_t, u_t, y_t, w_t] = sys.get_training_data();

        % Data-driven state feedback control
        K_h2_i = IndirectMethod(sys, Inf, y_t, u_t); % H2 (indirect)
        h2dType = "h2";
        if suboptimal
            h2dType = "h2s";
        end
        K_h2_d = DirectMethod(sys, Inf, y_t, u_t, h2dType); % H2 (direct)
        K_hinf_i = IndirectMethod(sys, gammaDesign, y_t, u_t); % H-inf (indirect)

        % Evaluate results (indirect H2)
        if check_stability(sys.sysd.A, sys.sysd.B, K_h2_i);
            % Compute linearized plant model(s)
            [P_ol, ~, ~, T_cl] = sys.get_plant(K_h2_i);

            % Compute disk-based stability margins
            MMIO = diskmargin(P_ol, -1*K_h2_i);
            DM_h2_i(n) = 2*MMIO.DiskMargin; % loop margin w/ simultaneous I/O variation

            % Compute singular values and H-infinity norm of the generalized plant
            SV = sigma(T_cl, sys.W);
            err = mag2db(max(SV(1,:))) - mag2db((1.5/gammaDesign));
            if abs(err) < errThreshDB % db
                SVmax_h2_i(n,:) = SV(1,:);
            end
        end

        % Evaluate results (direct H2)
        if check_stability(sys.sysd.A, sys.sysd.B, K_h2_d);
            % Compute linearized plant model(s)
            [P_ol, ~, ~, T_cl] = sys.get_plant(K_h2_d);

            % Compute disk-based stability margins
            MMIO = diskmargin(P_ol, -1*K_h2_d);
            DM_h2_d(n) = 2*MMIO.DiskMargin; % loop margin w/ simultaneous I/O variation

            % Compute singular values and H-infinity norm of the generalized plant
            SV = sigma(T_cl, sys.W);
            err = mag2db(max(SV(1,:))) - mag2db((1.5/gammaDesign));
            if abs(err) < errThreshDB % db
                SVmax_h2_d(n,:) = SV(1,:);
            end
        end

        % Evaluate results (indirect H-infinity)
        if check_stability(sys.sysd.A, sys.sysd.B, K_hinf_i);
            % Compute linearized plant model(s)
            [P_ol, ~, ~, T_cl] = sys.get_plant(K_hinf_i);

            % Compute disk-based stability margins
            MMIO = diskmargin(P_ol, -1*K_hinf_i);
            DM_hinf_i(n) = 2*MMIO.DiskMargin; % loop margin w/ simultaneous I/O variation

            % Compute singular values and H-infinity norm of the generalized plant
            SV = sigma(T_cl, sys.W);
            err = mag2db(max(SV(1,:))) - mag2db((1.5/gammaDesign));
            if abs(err) < errThreshDB % db
                SVmax_hinf_i(n,:) = SV(1,:);
            end
        end
    end

    figure(fignum); nexttile(tilenum); hold on;
    lh3_ = semilogx(sys.W, mag2db(SVmax_hinf_i*(1.5/gammaDesign)), 'k');
    for l = 1:size(lh3_)
        lh3_(l).Color(4) = 0.3;
    end
    lh1_ = semilogx(sys.W, mag2db(SVmax_h2_i*(1.5/gammaDesign)), 'r');
    for l = 1:size(lh1_)
        lh1_(l).Color(4) = 0.7;
    end
    lh2_ = semilogx(sys.W, mag2db(SVmax_h2_d*(1.5/gammaDesign)), 'b');
    for l = 1:size(lh2_)
        lh2_(l).Color(4) = 0.7;
    end

    % Dummy lines with no opacity and thick linewidths
    lh1 = semilogx(sys.W, -1e4*ones(size(sys.W)), 'r');
    lh1.LineWidth = 1;
    lh2 = semilogx(sys.W, -1e4*ones(size(sys.W)), 'b');
    lh2.LineWidth = 1;
    lh3 = semilogx(sys.W, -1e4*ones(size(sys.W)), 'k');
    lh3.LineWidth = 1;

    grid on; xlim([1e-3; pi/sys.dt]); ylim([-35 11]); set(gca,'XScale','log');
end

function [] = run_uncertainty_tests(fignum, numRuns, suboptimal)
    % Run for all three case studies
    scaleFactor = [0, 0.05, 0.1, 0.2];
    figure(fignum); clf; tl = tiledlayout(length(scaleFactor),3,'TileSpacing','Compact');
    %
    %   1  2  3  -----> 1
    %   4  5  6  -----> 2
    %   7  8  9  -----> 3
    %  10 11 12  -----> 4
    %
    ii = 1;
    for SF = scaleFactor
        test_and_plot_uncertainty(fignum, ii, "double-integrator", numRuns, SF, Inf, suboptimal);
        figure(fignum); nexttile(ii);
        ylabel('Magnitude (dB)','FontName','Helvetica Neue','FontSize',16);
        [lg, lh1, lh2, lh3] = test_and_plot_uncertainty(fignum, ii + 1, "cart-pole", numRuns, SF, Inf, suboptimal);
        test_and_plot_uncertainty(fignum, ii + 2, "robot-arm", numRuns, SF, Inf, suboptimal);
        ii = ii + 3;
    end

    ii = ii - 3;
    for jj = 0:2
        figure(fignum); nexttile(ii + jj);
        xlabel('Frequency (rad/s)','FontName','Helvetica Neue','FontSize',16);
    end
    figure(fignum); nexttile(ii + 1); ax = gca;
    legend([lg, lh1, lh2, lh3], '$\gamma$', '$\mathcal{H}_{2}$ (indirect)',...
        '$\mathcal{H}_{2}$ (direct)', '$\mathcal{H}_{\infty}$ (indirect)',...
        'Location','SouthOutside','Orientation','horizontal','Interpreter',...
        'latex','FontSize',16);

    figure(fignum); nexttile(1); title("Double Integrator",...
        'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
    figure(fignum); nexttile(2); title("Cart Pole",...
        'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
    figure(fignum); nexttile(3); title("Robot Arm",...
        'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
end

function [lg, lh1, lh2, lh3] = test_and_plot_noise(fignum, tilenum, caseStudy,...
    numRuns, scaleFactor, errThreshDB, suboptimal)

    % Testing data-driven control with measurement noise
    if caseStudy == "double-integrator"
        sys = DoubleIntegrator;
        capitalizedName = "Double Integrator";
    elseif caseStudy == "cart-pole"
        sys = CartPole;
        capitalizedName = "Cart Pole";
    elseif caseStudy == "robot-arm"
        sys = RobotArm;
        capitalizedName = "Robot Arm";
    end

    % Compute baseline state feedback controllers
    gammaDesign = minimaxPolicy(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, eps, 1e5, 1.5);
    [~, K_h2_b] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, Inf);
    [~, K_hinf_b] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, gammaDesign);
    sys.ctrl.gamma = gammaDesign;

    figure(fignum); nexttile(tilenum); cla; hold on;
    lg = semilogx(sys.W, mag2db(gammaDesign*(1.5/gammaDesign))*ones(1,length(sys.W)), 'k--');
    grid on; xlim([1e-3; pi/sys.dt]); 

    SVmax_h2_i = nan(numRuns, size(sys.W, 2));
    SVmax_h2_d = nan(numRuns, size(sys.W, 2));
    SVmax_hinf_i = nan(numRuns, size(sys.W, 2));

    DM_h2_i = zeros(numRuns, 1);
    DM_h2_d = zeros(numRuns, 1);
    DM_hinf_i = zeros(numRuns, 1);

    rng(1); % initialize the psuedo-random number generator

    % Scale the variance of zero-mean, additive measurement noise
    sys = sys.noise_variance(scaleFactor);

    for n = 1:numRuns
        disp([' Running test case ' num2str(n) ' of ' num2str(numRuns)]);

        % Gather training data from the full system, check feasiblity conditions
        [t_t, x_t, u_t, y_t, w_t] = sys.get_training_data();

        % Data-driven state feedback control
        K_h2_i = IndirectMethod(sys, Inf, y_t, u_t); % H2 (indirect)
        h2dType = "h2";
        if suboptimal
            h2dType = "h2s";
        end
        K_h2_d = DirectMethod(sys, Inf, y_t, u_t, h2dType); % H2 (direct)
        K_hinf_i = IndirectMethod(sys, gammaDesign, y_t, u_t); % H-inf (indirect)

        % Evaluate results (indirect H2)
        if check_stability(sys.sysd.A, sys.sysd.B, K_h2_i);
            % Compute linearized plant model(s)
            [P_ol, ~, ~, T_cl] = sys.get_plant(K_h2_i);

            % Compute disk-based stability margins
            MMIO = diskmargin(P_ol, -1*K_h2_i);
            DM_h2_i(n) = 2*MMIO.DiskMargin; % loop margin w/ simultaneous I/O variation

            % Compute singular values and H-infinity norm of the generalized plant
            SV = sigma(T_cl, sys.W);
            err = mag2db(max(SV(1,:))) - mag2db((1.5/gammaDesign));
            if abs(err) < errThreshDB % db
                SVmax_h2_i(n,:) = SV(1,:);
            end
        end

        % Evaluate results (direct H2)
        if check_stability(sys.sysd.A, sys.sysd.B, K_h2_d);
            % Compute linearized plant model(s)
            [P_ol, ~, ~, T_cl] = sys.get_plant(K_h2_d);

            % Compute disk-based stability margins
            MMIO = diskmargin(P_ol, -1*K_h2_d);
            DM_h2_d(n) = 2*MMIO.DiskMargin; % loop margin w/ simultaneous I/O variation

            % Compute singular values and H-infinity norm of the generalized plant
            SV = sigma(T_cl, sys.W);
            err = mag2db(max(SV(1,:))) - mag2db((1.5/gammaDesign));
            if abs(err) < errThreshDB % db
                SVmax_h2_d(n,:) = SV(1,:);
            end
        end

        % Evaluate results (indirect H-infinity)
        if check_stability(sys.sysd.A, sys.sysd.B, K_hinf_i);
            % Compute linearized plant model(s)
            [P_ol, ~, ~, T_cl] = sys.get_plant(K_hinf_i);

            % Compute disk-based stability margins
            MMIO = diskmargin(P_ol, -1*K_hinf_i);
            DM_hinf_i(n) = 2*MMIO.DiskMargin; % loop margin w/ simultaneous I/O variation

            % Compute singular values and H-infinity norm of the generalized plant
            SV = sigma(T_cl, sys.W);
            err = mag2db(max(SV(1,:))) - mag2db((1.5/gammaDesign));
            if abs(err) < errThreshDB % db
                SVmax_hinf_i(n,:) = SV(1,:);
            end
        end
    end

    figure(fignum); nexttile(tilenum); hold on;
    lh3_ = semilogx(sys.W, mag2db(SVmax_hinf_i*(1.5/gammaDesign)), 'k');
    for l = 1:size(lh3_)
        lh3_(l).Color(4) = 0.3;
    end
    lh1_ = semilogx(sys.W, mag2db(SVmax_h2_i*(1.5/gammaDesign)), 'r');
    for l = 1:size(lh1_)
        lh1_(l).Color(4) = 0.7;
    end
    lh2_ = semilogx(sys.W, mag2db(SVmax_h2_d*(1.5/gammaDesign)), 'b');
    for l = 1:size(lh2_)
        lh2_(l).Color(4) = 0.7;
    end

    % Dummy lines with no opacity and thick linewidths
    lh1 = semilogx(sys.W, -1e4*ones(size(sys.W)), 'r');
    lh1.LineWidth = 1;
    lh2 = semilogx(sys.W, -1e4*ones(size(sys.W)), 'b');
    lh2.LineWidth = 1;
    lh3 = semilogx(sys.W, -1e4*ones(size(sys.W)), 'k');
    lh3.LineWidth = 1;

    grid on; xlim([1e-3; pi/sys.dt]); ylim([-35 11]); set(gca,'XScale','log');
end

function [] = run_noise_tests(fignum, numRuns, suboptimal)
    % Run for all three case studies
    scaleFactor = [0, 0.05, 0.1, 0.2];
    figure(fignum); clf; tl = tiledlayout(length(scaleFactor),3,'TileSpacing','Compact');
    %
    %   1  2  3  -----> 1
    %   4  5  6  -----> 2
    %   7  8  9  -----> 3
    %  10 11 12  -----> 4
    %
    ii = 1;
    for SF = scaleFactor
        test_and_plot_noise(fignum, ii, "double-integrator", numRuns, SF, Inf, suboptimal);
        figure(fignum); nexttile(ii);
        ylabel('Magnitude (dB)','FontName','Helvetica Neue','FontSize',16);
        [lg, lh1, lh2, lh3] = test_and_plot_noise(fignum, ii + 1, "cart-pole", numRuns, SF, Inf, suboptimal);
        test_and_plot_noise(fignum, ii + 2, "robot-arm", numRuns, SF, Inf, suboptimal);
        ii = ii + 3;
    end

    ii = ii - 3;
    for jj = 0:2
        figure(fignum); nexttile(ii + jj);
        xlabel('Frequency (rad/s)','FontName','Helvetica Neue','FontSize',16);
    end
    figure(fignum); nexttile(ii + 1); ax = gca;
    legend([lg, lh1, lh2, lh3], '$\gamma$', '$\mathcal{H}_{2}$ (indirect)',...
        '$\mathcal{H}_{2}$ (direct, suboptimal)', '$\mathcal{H}_{\infty}$ (indirect)',...
        'Location','SouthOutside','Orientation','horizontal','Interpreter',...
        'latex','FontSize',16);

    figure(fignum); nexttile(1); title("Double Integrator",...
        'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
    figure(fignum); nexttile(2); title("Cart Pole",...
        'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
    figure(fignum); nexttile(3); title("Robot Arm",...
        'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
end

function [] = fix_ylimits(fignum, ylim1, ylim2, ylim3)
    for ii = [1, 4, 7, 10]
        figure(fignum); nexttile(ii); ylim(ylim1);
        figure(fignum); nexttile(ii+1); ylim(ylim2);
        figure(fignum); nexttile(ii+2); ylim(ylim3);
    end
end
