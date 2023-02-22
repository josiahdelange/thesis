function [] = run_all_overall_trends(fignums, barGraphs, saveFigs)
    %% Runs all the uncertainty and noise tests to get overall trends
    outputDir = '/Users/josiah/ms-thesis/latex/figures';

    if nargin < 2
        barGraphs = true;
    end
    if nargin < 3
        saveFigs = true;
    end
    if barGraphs
        ls_opt = {'$\mathcal{H}_{2}$ (baseline)','$\mathcal{H}_{\infty}$ (baseline)'...
            '$\mathcal{H}_{2}$ (indirect)', '$\mathcal{H}_{2}$ (direct)',...
            '$\mathcal{H}_{\infty}$ (indirect)'};
        ls_subopt = {'$\mathcal{H}_{2}$ (baseline)','$\mathcal{H}_{\infty}$ (baseline)'...
            '$\mathcal{H}_{2}$ (indirect)', '$\mathcal{H}_{2}$ (direct, suboptimal)',...
            '$\mathcal{H}_{\infty}$ (indirect)'};
        make_single_plot = @(caseStudy, testDir, fignum, uncertaintyTest, suboptimal) ...
            make_single_plot_bar(caseStudy, testDir, fignum, uncertaintyTest, suboptimal);
    else
        ls_opt = {'$\mathcal{H}_{2}$ (baseline)',...
            '$\mathcal{H}_{2}$ (indirect)', '$\mathcal{H}_{2}$ (direct)',...
            '$\mathcal{H}_{\infty}$ (baseline)',...
            '$\mathcal{H}_{\infty}$ (indirect)'};
        ls_subopt = {'$\mathcal{H}_{2}$ (baseline)',...
            '$\mathcal{H}_{2}$ (indirect)', '$\mathcal{H}_{2}$ (direct, suboptimal)',...
            '$\mathcal{H}_{\infty}$ (baseline)',...
            '$\mathcal{H}_{\infty}$ (indirect)'};
        make_single_plot = @(caseStudy, testDir, fignum, uncertaintyTest, suboptimal) ...
            make_single_plot_line(caseStudy, testDir, fignum, uncertaintyTest);
    end

    %% Canonical/optimal LMI formulation for H2 direct
    tic;

    % Overall trends: parameter uncertainty
    testDir1 = test_uncertainty("double-integrator", 100, false, false);
    testDir2 = test_uncertainty("cart-pole", 100, false, false);
    testDir3 = test_uncertainty("robot-arm", 100, false, false);

    figure(fignums(1)); tl = tiledlayout(5,3,'TileSpacing','Compact');
    make_single_plot("double-integrator", testDir1, fignums(1), true, false);
    make_single_plot("cart-pole", testDir2, fignums(1), true, false);
    make_single_plot("robot-arm", testDir3, fignums(1), true, false);

    figure(fignums(1)); nexttile(1); ylabel('Stable (%)','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(1)); nexttile(4); ylabel('Disk Margin','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(1)); nexttile(7); ylabel('H_{\infty} Norm','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(1)); nexttile(10); ylabel('Total Cost','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(1)); nexttile(13); ylabel('Total Effort','FontName','Helvetica Neue','FontSize',16);
    for ii = 13:15
        figure(fignums(1)); nexttile(ii);
        xlabel('Scale Factor (%)','FontName','Helvetica Neue','FontSize',16);
    end
    figure(fignums(1)); nexttile(14); ax = gca;
    legend(ax, ls_opt, 'Location','SouthOutside',...
        'Orientation','horizontal','Interpreter','latex','FontSize',16);
    normalize_ylims(fignums(1));


    % Overall trends: measurement noise
    testDir4 = test_noise("double-integrator", 100, false, false);
    testDir5 = test_noise("cart-pole", 100, false, false);
    testDir6 = test_noise("robot-arm", 100, false, false);

    figure(fignums(2)); tl = tiledlayout(5,3,'TileSpacing','Compact');
    make_single_plot("double-integrator", testDir4, fignums(2), false, false);
    make_single_plot("cart-pole", testDir5, fignums(2), false, false);
    make_single_plot("robot-arm", testDir6, fignums(2), false, false);

    figure(fignums(2)); nexttile(1); ylabel('Stable (%)','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(2)); nexttile(4); ylabel('Disk Margin','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(2)); nexttile(7); ylabel('H_{\infty} Norm','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(2)); nexttile(10); ylabel('Total Cost','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(2)); nexttile(13); ylabel('Total Effort','FontName','Helvetica Neue','FontSize',16);
    for ii = 13:15
        figure(fignums(2)); nexttile(ii);
        xlabel('Scale Factor (%)','FontName','Helvetica Neue','FontSize',16);
    end
    figure(fignums(2)); nexttile(14); ax = gca;
    legend(ax, ls_opt, 'Location','SouthOutside',...
        'Orientation','horizontal','Interpreter','latex','FontSize',16);
    normalize_ylims(fignums(2));

    toc;

    %% Adjusted/suboptimal LMI formulation for H2 direct
    tic;

    % Overall trends: parameter uncertainty
    testDir7 = test_uncertainty("double-integrator", 100, false, true);
    testDir8 = test_uncertainty("cart-pole", 100, false, true);
    testDir9 = test_uncertainty("robot-arm", 100, false, true);

    figure(fignums(3)); tl = tiledlayout(5,3,'TileSpacing','Compact');
    make_single_plot("double-integrator", testDir7, fignums(3), true, true);
    make_single_plot("cart-pole", testDir8, fignums(3), true, true);
    make_single_plot("robot-arm", testDir9, fignums(3), true, true);

    figure(fignums(3)); nexttile(1); ylabel('Stable (%)','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(3)); nexttile(4); ylabel('Disk Margin','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(3)); nexttile(7); ylabel('H_{\infty} Norm','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(3)); nexttile(10); ylabel('Total Cost','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(3)); nexttile(13); ylabel('Total Effort','FontName','Helvetica Neue','FontSize',16);
    for ii = 13:15
        figure(fignums(3)); nexttile(ii);
        xlabel('Scale Factor (%)','FontName','Helvetica Neue','FontSize',16);
    end
    figure(fignums(3)); nexttile(14); ax = gca;
    legend(ax, ls_subopt, 'Location','SouthOutside',...
        'Orientation','horizontal','Interpreter','latex','FontSize',16);
    normalize_ylims(fignums(3));

    % Overall trends: measurement noise
    testDir10 = test_noise("double-integrator", 100, false, true);
    testDir11 = test_noise("cart-pole", 100, false, true);
    testDir12 = test_noise("robot-arm", 100, false, true);

    figure(fignums(4)); tl = tiledlayout(5,3,'TileSpacing','Compact');
    make_single_plot("double-integrator", testDir10, fignums(4), false, true);
    make_single_plot("cart-pole", testDir11, fignums(4), false, true);
    make_single_plot("robot-arm", testDir12, fignums(4), false, true);

    figure(fignums(4)); nexttile(1); ylabel('Stable (%)','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(4)); nexttile(4); ylabel('Disk Margin','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(4)); nexttile(7); ylabel('H_{\infty} Norm','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(4)); nexttile(10); ylabel('Total Cost','FontName','Helvetica Neue','FontSize',16);
    figure(fignums(4)); nexttile(13); ylabel('Total Effort','FontName','Helvetica Neue','FontSize',16);
    for ii = 13:15
        figure(fignums(4)); nexttile(ii);
        xlabel('Scale Factor (%)','FontName','Helvetica Neue','FontSize',16);
    end
    figure(fignums(4)); nexttile(14); ax = gca;
    legend(ax, ls_subopt, 'Location','SouthOutside',...
        'Orientation','horizontal','Interpreter','latex','FontSize',16);
    normalize_ylims(fignums(4));

    toc;

    %% Save figures
    if saveFigs
        if barGraphs
            disp('Saving .fig files..');
            savefig(fignums(1), fullfile(outputDir, 'overall_trends_uncertainty_opt_bar'));
            savefig(fignums(2), fullfile(outputDir, 'overall_trends_noise_opt_bar'));
            savefig(fignums(3), fullfile(outputDir, 'overall_trends_uncertainty_subopt_bar'));
            savefig(fignums(4), fullfile(outputDir, 'overall_trends_noise_subopt_bar'));
            disp('...done.');
        
            disp('Saving .png files..');
            saveas(fignums(1), fullfile(outputDir, 'overall_trends_uncertainty_opt_bar.png'));
            saveas(fignums(2), fullfile(outputDir, 'overall_trends_noise_opt_bar.png'));
            saveas(fignums(3), fullfile(outputDir, 'overall_trends_uncertainty_subopt_bar.png'));
            saveas(fignums(4), fullfile(outputDir, 'overall_trends_noise_subopt_bar.png'));
            disp('...done.');
        else
            disp('Saving .fig files..');
            savefig(fignums(1), fullfile(outputDir, 'overall_trends_uncertainty_opt'));
            savefig(fignums(2), fullfile(outputDir, 'overall_trends_noise_opt'));
            savefig(fignums(3), fullfile(outputDir, 'overall_trends_uncertainty_subopt'));
            savefig(fignums(4), fullfile(outputDir, 'overall_trends_noise_subopt'));
            disp('...done.');
        
            disp('Saving .png files..');
            saveas(fignums(1), fullfile(outputDir, 'overall_trends_uncertainty_opt.png'));
            saveas(fignums(2), fullfile(outputDir, 'overall_trends_noise_opt.png'));
            saveas(fignums(3), fullfile(outputDir, 'overall_trends_uncertainty_subopt.png'));
            saveas(fignums(4), fullfile(outputDir, 'overall_trends_noise_subopt.png'));
            disp('...done.');
        end
    end
end

%% Subroutines
function [testDir] = test_uncertainty(caseStudy, numRuns, makePlots, suboptimal)
    % Testing data-driven control with inertial uncertainty
    if caseStudy == "double-integrator"
        sys = DoubleIntegrator;
    elseif caseStudy == "cart-pole"
        sys = CartPole;
    elseif caseStudy == "robot-arm"
        sys = RobotArm;
    end

    % Create output directory
    testDir = fullfile(pwd, [['logs/' regexprep(sys.name_str, ' ', '-') '/uncertainty-']...
        + string(datetime('now', 'Format', 'yyyy-MM-dd-HH-mm-ss'))])
    if ~isfolder(testDir)
        mkdir(testDir)
    end

    % Compute baseline state feedback controllers
    gammaDesign = minimaxPolicy(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, eps, 1e5, 1.5);
    [~, K_h2_b] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, Inf);
    [~, K_hinf_b] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, gammaDesign);
    sys.ctrl.gamma = gammaDesign;

    % LQR optimal cost, control effort, disk margin
    [t, x, u, ~, ~, w] = sys.simulate(sys.x0, sys.u0, sys.N, @(t,x) K_h2_b*x);
    [J_star, Ju_star] = sys.compute_cost(t, u, x, w);
    J_star = J_star(end); Ju_star = Ju_star(end);
    [P_ol, ~, ~, T_cl] = sys.get_plant(K_h2_b);
    MMIO = diskmargin(P_ol, -1*K_h2_b);
    DM_opt = 2*MMIO.DiskMargin; % loop margin w/ simultaneous I/O variation

    % Sweep through different noise variances: 0% - 60% of nominal value
    scaleFactor = 0:0.1:0.6;

    Stable_H2_baseline = zeros(numRuns, length(scaleFactor));
    DiskMargin_H2_baseline = zeros(numRuns, length(scaleFactor));
    HInfinityNorm_H2_baseline = zeros(numRuns, length(scaleFactor));
    LQCost_H2_baseline = zeros(numRuns, length(scaleFactor));
    LQGammaCost_H2_baseline = zeros(numRuns, length(scaleFactor));
    ControlEffort_H2_baseline = zeros(numRuns, length(scaleFactor));
    DeltaInfNorm_H2_baseline = zeros(numRuns, length(scaleFactor));

    Stable_H2_indirect = zeros(numRuns, length(scaleFactor));
    DiskMargin_H2_indirect = zeros(numRuns, length(scaleFactor));
    HInfinityNorm_H2_indirect = zeros(numRuns, length(scaleFactor));
    LQCost_H2_indirect = zeros(numRuns, length(scaleFactor));
    LQGammaCost_H2_indirect = zeros(numRuns, length(scaleFactor));
    ControlEffort_H2_indirect = zeros(numRuns, length(scaleFactor));
    DeltaInfNorm_H2_indirect = zeros(numRuns, length(scaleFactor));

    Stable_H2_direct = zeros(numRuns, length(scaleFactor));
    DiskMargin_H2_direct = zeros(numRuns, length(scaleFactor));
    HInfinityNorm_H2_direct = zeros(numRuns, length(scaleFactor));
    LQCost_H2_direct = zeros(numRuns, length(scaleFactor));
    LQGammaCost_H2_direct = zeros(numRuns, length(scaleFactor));
    ControlEffort_H2_direct = zeros(numRuns, length(scaleFactor));
    DeltaInfNorm_H2_direct = zeros(numRuns, length(scaleFactor));

    Stable_HInf_baseline = zeros(numRuns, length(scaleFactor));
    DiskMargin_HInf_baseline = zeros(numRuns, length(scaleFactor));
    HInfinityNorm_HInf_baseline = zeros(numRuns, length(scaleFactor));
    LQCost_HInf_baseline = zeros(numRuns, length(scaleFactor));
    LQGammaCost_HInf_baseline = zeros(numRuns, length(scaleFactor));
    ControlEffort_HInf_baseline = zeros(numRuns, length(scaleFactor));
    DeltaInfNorm_HInf_baseline = zeros(numRuns, length(scaleFactor));

    Stable_HInf_indirect = zeros(numRuns, length(scaleFactor));
    DiskMargin_HInf_indirect = zeros(numRuns, length(scaleFactor));
    HInfinityNorm_HInf_indirect = zeros(numRuns, length(scaleFactor));
    LQCost_HInf_indirect = zeros(numRuns, length(scaleFactor));
    LQGammaCost_HInf_indirect = zeros(numRuns, length(scaleFactor));
    ControlEffort_HInf_indirect = zeros(numRuns, length(scaleFactor));
    DeltaInfNorm_HInf_indirect = zeros(numRuns, length(scaleFactor));

    Stable_Lyap_direct = zeros(numRuns, length(scaleFactor));
    DiskMargin_Lyap_direct = zeros(numRuns, length(scaleFactor));
    HInfinityNorm_Lyap_direct = zeros(numRuns, length(scaleFactor));
    LQCost_Lyap_direct = zeros(numRuns, length(scaleFactor));
    LQGammaCost_Lyap_direct = zeros(numRuns, length(scaleFactor));
    ControlEffort_Lyap_direct = zeros(numRuns, length(scaleFactor));
    DeltaInfNorm_Lyap_direct = zeros(numRuns, length(scaleFactor));

    for ii = 1:length(scaleFactor)
        disp(['Running scenario ' num2str(ii) ' of ' num2str(length(scaleFactor))]);
        rng(1); % initialize the psuedo-random number generator
        for n = 1:numRuns
            disp([' Running test case ' num2str(n) ' of ' num2str(numRuns)]);

            % Scale the variance of zero-mean, single-parameter uncertainty
            sys = sys.parameter_uncertainty(scaleFactor(ii));

            % Gather training data from the full system, check feasiblity conditions
            disp('  Gathering training data');
            [t_t, x_t, u_t, y_t, w_t] = sys.get_training_data();

            % Data-driven state feedback control
            disp('  Computing data-driven control');
            K_h2_i = IndirectMethod(sys, Inf, y_t, u_t); % H2 (indirect)
            h2dType = "h2";
            if suboptimal
                h2dType = "h2s";
            end
            K_h2_d = DirectMethod(sys, Inf, y_t, u_t, h2dType); % H2 (direct)
            K_hinf_i = IndirectMethod(sys, gammaDesign, y_t, u_t); % H-inf (indirect)
            K_lyap_d = DirectMethod(sys, Inf, y_t, u_t, "lyapunov"); % Lyaounov (direct)

            % Evaluate results (baseline H2)
            disp('  Testing baseline H2');
            [Stable, DiskMargin, HInfinityNorm, LQCost, LQGammaCost, ControlEffort, DeltaInfNorm] = sys.run_test(K_h2_b);
            Stable_H2_baseline(n, ii) = Stable;
            DiskMargin_H2_baseline(n, ii) = DiskMargin/DM_opt;
            HInfinityNorm_H2_baseline(n, ii) = HInfinityNorm/gammaDesign;
            LQCost_H2_baseline(n, ii) = LQCost/J_star;
            LQGammaCost_H2_baseline(n, ii) = LQGammaCost;
            ControlEffort_H2_baseline(n, ii) = ControlEffort/Ju_star;
            DeltaInfNorm_H2_baseline(n, ii) = DeltaInfNorm;

            % Evaluate results (indirect H2)
            disp('  Testing indirect H2');
            [Stable, DiskMargin, HInfinityNorm, LQCost, LQGammaCost, ControlEffort, DeltaInfNorm] = sys.run_test(K_h2_i);
            Stable_H2_indirect(n, ii) = Stable;
            DiskMargin_H2_indirect(n, ii) = DiskMargin/DM_opt;
            HInfinityNorm_H2_indirect(n, ii) = HInfinityNorm/gammaDesign;
            LQCost_H2_indirect(n, ii) = LQCost/J_star;
            LQGammaCost_H2_indirect(n, ii) = LQGammaCost;
            ControlEffort_H2_indirect(n, ii) = ControlEffort/Ju_star;
            DeltaInfNorm_H2_indirect(n, ii) = DeltaInfNorm;

            % Evaluate results (direct H2)
            disp('  Testing direct H2');
            [Stable, DiskMargin, HInfinityNorm, LQCost, LQGammaCost, ControlEffort, DeltaInfNorm] = sys.run_test(K_h2_d);
            Stable_H2_direct(n, ii) = Stable;
            DiskMargin_H2_direct(n, ii) = DiskMargin/DM_opt;
            HInfinityNorm_H2_direct(n, ii) = HInfinityNorm/gammaDesign;
            LQCost_H2_direct(n, ii) = LQCost/J_star;
            LQGammaCost_H2_direct(n, ii) = LQGammaCost;
            ControlEffort_H2_direct(n, ii) = ControlEffort/Ju_star;
            DeltaInfNorm_H2_direct(n, ii) = DeltaInfNorm;

            % Evaluate results (baseline H-infinity)
            disp('  Testing baseline Hinf');
            [Stable, DiskMargin, HInfinityNorm, LQCost, LQGammaCost, ControlEffort, DeltaInfNorm] = sys.run_test(K_hinf_b);
            Stable_HInf_baseline(n, ii) = Stable;
            DiskMargin_HInf_baseline(n, ii) = DiskMargin/DM_opt;
            HInfinityNorm_HInf_baseline(n, ii) = HInfinityNorm/gammaDesign;
            LQCost_HInf_baseline(n, ii) = LQCost/J_star;
            LQGammaCost_HInf_baseline(n, ii) = LQGammaCost;
            ControlEffort_HInf_baseline(n, ii) = ControlEffort/Ju_star;
            DeltaInfNorm_HInf_baseline(n, ii) = DeltaInfNorm;

            % Evaluate results (indirect H-infinity)
            disp('  Testing indirect Hinf');
            [Stable, DiskMargin, HInfinityNorm, LQCost, LQGammaCost, ControlEffort, DeltaInfNorm] = sys.run_test(K_hinf_i);
            Stable_HInf_indirect(n, ii) = Stable;
            DiskMargin_HInf_indirect(n, ii) = DiskMargin/DM_opt;
            HInfinityNorm_HInf_indirect(n, ii) = HInfinityNorm/gammaDesign;
            LQCost_HInf_indirect(n, ii) = LQCost/J_star;
            LQGammaCost_HInf_indirect(n, ii) = LQGammaCost;
            ControlEffort_HInf_indirect(n, ii) = ControlEffort/Ju_star;
            DeltaInfNorm_HInf_indirect(n, ii) = DeltaInfNorm;

            % Evaluate results (direct Lyapunov)
            disp('  Skipping direct Lyapunov');
            %[Stable, DiskMargin, HInfinityNorm, LQCost, LQGammaCost, ControlEffort, DeltaInfNorm] = sys.run_test(K_lyap_d);
            %Stable_Lyap_direct(n, ii) = Stable;
            %DiskMargin_Lyap_direct(n, ii) = DiskMargin/DM_opt;
            %HInfinityNorm_Lyap_direct(n, ii) = HInfinityNorm/gammaDesign;
            %LQCost_Lyap_direct(n, ii) = LQCost/J_star;
            %LQGammaCost_Lyap_direct(n, ii) = LQGammaCost;
            %ControlEffort_Lyap_direct(n, ii) = ControlEffort/Ju_star;
            %DeltaInfNorm_Lyap_direct(n, ii) = DeltaInfNorm;
        end
    end

    % Store all data to disk, to create tables
    save(fullfile(testDir, 'overall_trends'));
end

function [testDir] = test_noise(caseStudy, numRuns, makePlots, suboptimal)
    % Testing data-driven control with measurement noise
    if caseStudy == "double-integrator"
        sys = DoubleIntegrator;
    elseif caseStudy == "cart-pole"
        sys = CartPole;
    elseif caseStudy == "robot-arm"
        sys = RobotArm;
    end

    % Create output directory
    testDir = fullfile(pwd, [['logs/' regexprep(sys.name_str, ' ', '-') '/noise-']...
        + string(datetime('now', 'Format', 'yyyy-MM-dd-HH-mm-ss'))])
    if ~isfolder(testDir)
        mkdir(testDir)
    end

    % Compute baseline state feedback controllers
    gammaDesign = minimaxPolicy(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, eps, 1e5, 1.5);
    [~, K_h2_b] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, Inf);
    [~, K_hinf_b] = solveDGARE(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, gammaDesign);
    sys.ctrl.gamma = gammaDesign;

    % LQR optimal cost, control effort, disk margin
    [t, x, u, ~, ~, w] = sys.simulate(sys.x0, sys.u0, sys.N, @(t,x) K_h2_b*x);
    [J_star, Ju_star] = sys.compute_cost(t, u, x, w);
    J_star = J_star(end); Ju_star = Ju_star(end);
    [P_ol, ~, ~, T_cl] = sys.get_plant(K_h2_b);
    MMIO = diskmargin(P_ol, -1*K_h2_b);
    DM_opt = 2*MMIO.DiskMargin; % loop margin w/ simultaneous I/O variation

    % Sweep through different noise variances: 0% - 60% of nominal value
    scaleFactor = 0:0.1:0.6;

    Stable_H2_baseline = zeros(numRuns, length(scaleFactor));
    DiskMargin_H2_baseline = zeros(numRuns, length(scaleFactor));
    HInfinityNorm_H2_baseline = zeros(numRuns, length(scaleFactor));
    LQCost_H2_baseline = zeros(numRuns, length(scaleFactor));
    LQGammaCost_H2_baseline = zeros(numRuns, length(scaleFactor));
    ControlEffort_H2_baseline = zeros(numRuns, length(scaleFactor));
    DeltaInfNorm_H2_baseline = zeros(numRuns, length(scaleFactor));

    Stable_H2_indirect = zeros(numRuns, length(scaleFactor));
    DiskMargin_H2_indirect = zeros(numRuns, length(scaleFactor));
    HInfinityNorm_H2_indirect = zeros(numRuns, length(scaleFactor));
    LQCost_H2_indirect = zeros(numRuns, length(scaleFactor));
    LQGammaCost_H2_indirect = zeros(numRuns, length(scaleFactor));
    ControlEffort_H2_indirect = zeros(numRuns, length(scaleFactor));
    DeltaInfNorm_H2_indirect = zeros(numRuns, length(scaleFactor));

    Stable_H2_direct = zeros(numRuns, length(scaleFactor));
    DiskMargin_H2_direct = zeros(numRuns, length(scaleFactor));
    HInfinityNorm_H2_direct = zeros(numRuns, length(scaleFactor));
    LQCost_H2_direct = zeros(numRuns, length(scaleFactor));
    LQGammaCost_H2_direct = zeros(numRuns, length(scaleFactor));
    ControlEffort_H2_direct = zeros(numRuns, length(scaleFactor));
    DeltaInfNorm_H2_direct = zeros(numRuns, length(scaleFactor));

    Stable_HInf_baseline = zeros(numRuns, length(scaleFactor));
    DiskMargin_HInf_baseline = zeros(numRuns, length(scaleFactor));
    HInfinityNorm_HInf_baseline = zeros(numRuns, length(scaleFactor));
    LQCost_HInf_baseline = zeros(numRuns, length(scaleFactor));
    LQGammaCost_HInf_baseline = zeros(numRuns, length(scaleFactor));
    ControlEffort_HInf_baseline = zeros(numRuns, length(scaleFactor));
    DeltaInfNorm_HInf_baseline = zeros(numRuns, length(scaleFactor));

    Stable_HInf_indirect = zeros(numRuns, length(scaleFactor));
    DiskMargin_HInf_indirect = zeros(numRuns, length(scaleFactor));
    HInfinityNorm_HInf_indirect = zeros(numRuns, length(scaleFactor));
    LQCost_HInf_indirect = zeros(numRuns, length(scaleFactor));
    LQGammaCost_HInf_indirect = zeros(numRuns, length(scaleFactor));
    ControlEffort_HInf_indirect = zeros(numRuns, length(scaleFactor));
    DeltaInfNorm_HInf_indirect = zeros(numRuns, length(scaleFactor));

    Stable_Lyap_direct = zeros(numRuns, length(scaleFactor));
    DiskMargin_Lyap_direct = zeros(numRuns, length(scaleFactor));
    HInfinityNorm_Lyap_direct = zeros(numRuns, length(scaleFactor));
    LQCost_Lyap_direct = zeros(numRuns, length(scaleFactor));
    LQGammaCost_Lyap_direct = zeros(numRuns, length(scaleFactor));
    ControlEffort_Lyap_direct = zeros(numRuns, length(scaleFactor));
    DeltaInfNorm_Lyap_direct = zeros(numRuns, length(scaleFactor));

    for ii = 1:length(scaleFactor)
        disp(['Running scenario ' num2str(ii) ' of ' num2str(length(scaleFactor))]);
        rng(1); % initialize the psuedo-random number generator

        % Scale the variance of zero-mean, additive measurement noise
        sys = sys.noise_variance(scaleFactor(ii));

        for n = 1:numRuns
            disp([' Running test case ' num2str(n) ' of ' num2str(numRuns)]);

            % Gather training data from the full system, check feasiblity conditions
            disp('  Gathering training data');
            [t_t, x_t, u_t, y_t, w_t] = sys.get_training_data();

            % Data-driven state feedback control
            disp('  Computing data-driven control');
            K_h2_i = IndirectMethod(sys, Inf, y_t, u_t); % H2 (indirect)
            h2dType = "h2";
            if suboptimal
                h2dType = "h2s";
            end
            K_h2_d = DirectMethod(sys, Inf, y_t, u_t, h2dType); % H2 (direct)
            K_hinf_i = IndirectMethod(sys, gammaDesign, y_t, u_t); % H-inf (indirect)
            K_lyap_d = DirectMethod(sys, Inf, y_t, u_t, "lyapunov"); % Lyaounov (direct)

            % Evaluate results (baseline H2)
            disp('  Testing baseline H2');
            [Stable, DiskMargin, HInfinityNorm, LQCost, LQGammaCost, ControlEffort, DeltaInfNorm] = sys.run_test(K_h2_b);
            Stable_H2_baseline(n, ii) = Stable;
            DiskMargin_H2_baseline(n, ii) = DiskMargin/DM_opt;
            HInfinityNorm_H2_baseline(n, ii) = HInfinityNorm/gammaDesign;
            LQCost_H2_baseline(n, ii) = LQCost/J_star;
            LQGammaCost_H2_baseline(n, ii) = LQGammaCost;
            ControlEffort_H2_baseline(n, ii) = ControlEffort/Ju_star;
            DeltaInfNorm_H2_baseline(n, ii) = DeltaInfNorm;

            % Evaluate results (indirect H2)
            disp('  Testing indirect H2');
            [Stable, DiskMargin, HInfinityNorm, LQCost, LQGammaCost, ControlEffort, DeltaInfNorm] = sys.run_test(K_h2_i);
            Stable_H2_indirect(n, ii) = Stable;
            DiskMargin_H2_indirect(n, ii) = DiskMargin/DM_opt;
            HInfinityNorm_H2_indirect(n, ii) = HInfinityNorm/gammaDesign;
            LQCost_H2_indirect(n, ii) = LQCost/J_star;
            LQGammaCost_H2_indirect(n, ii) = LQGammaCost;
            ControlEffort_H2_indirect(n, ii) = ControlEffort/Ju_star;
            DeltaInfNorm_H2_indirect(n, ii) = DeltaInfNorm;

            % Evaluate results (direct H2)
            disp('  Testing direct H2');
            [Stable, DiskMargin, HInfinityNorm, LQCost, LQGammaCost, ControlEffort, DeltaInfNorm] = sys.run_test(K_h2_d);
            Stable_H2_direct(n, ii) = Stable;
            DiskMargin_H2_direct(n, ii) = DiskMargin/DM_opt;
            HInfinityNorm_H2_direct(n, ii) = HInfinityNorm/gammaDesign;
            LQCost_H2_direct(n, ii) = LQCost/J_star;
            LQGammaCost_H2_direct(n, ii) = LQGammaCost;
            ControlEffort_H2_direct(n, ii) = ControlEffort/Ju_star;
            DeltaInfNorm_H2_direct(n, ii) = DeltaInfNorm;

            % Evaluate results (baseline H-infinity)
            disp('  Testing baseline Hinf');
            [Stable, DiskMargin, HInfinityNorm, LQCost, LQGammaCost, ControlEffort, DeltaInfNorm] = sys.run_test(K_hinf_b);
            Stable_HInf_baseline(n, ii) = Stable;
            DiskMargin_HInf_baseline(n, ii) = DiskMargin/DM_opt;
            HInfinityNorm_HInf_baseline(n, ii) = HInfinityNorm/gammaDesign;
            LQCost_HInf_baseline(n, ii) = LQCost/J_star;
            LQGammaCost_HInf_baseline(n, ii) = LQGammaCost;
            ControlEffort_HInf_baseline(n, ii) = ControlEffort/Ju_star;
            DeltaInfNorm_HInf_baseline(n, ii) = DeltaInfNorm;

            % Evaluate results (indirect H-infinity)
            disp('  Testing indirect Hinf');
            [Stable, DiskMargin, HInfinityNorm, LQCost, LQGammaCost, ControlEffort, DeltaInfNorm] = sys.run_test(K_hinf_i);
            Stable_HInf_indirect(n, ii) = Stable;
            DiskMargin_HInf_indirect(n, ii) = DiskMargin/DM_opt;
            HInfinityNorm_HInf_indirect(n, ii) = HInfinityNorm/gammaDesign;
            LQCost_HInf_indirect(n, ii) = LQCost/J_star;
            LQGammaCost_HInf_indirect(n, ii) = LQGammaCost;
            ControlEffort_HInf_indirect(n, ii) = ControlEffort/Ju_star;
            DeltaInfNorm_HInf_indirect(n, ii) = DeltaInfNorm;

            % Evaluate results (direct H2)
            disp('  Skipping direct Lyapunov');
            %[Stable, DiskMargin, HInfinityNorm, LQCost, LQGammaCost, ControlEffort, DeltaInfNorm] = sys.run_test(K_lyap_d);
            %Stable_Lyap_direct(n, ii) = Stable;
            %DiskMargin_Lyap_direct(n, ii) = DiskMargin/DM_opt;
            %HInfinityNorm_Lyap_direct(n, ii) = HInfinityNorm/gammaDesign;
            %LQCost_Lyap_direct(n, ii) = LQCost/J_star;
            %LQGammaCost_Lyap_direct(n, ii) = LQGammaCost;
            %ControlEffort_Lyap_direct(n, ii) = ControlEffort/Ju_star;
            %DeltaInfNorm_Lyap_direct(n, ii) = DeltaInfNorm;
        end
    end

    % Store all data to disk, to create tables
    save(fullfile(testDir, 'overall_trends'));
end

function [] = make_single_plot_bar(caseStudy, testDir, fignum, uncertaintyTest, suboptimal)
    clearvars -except testDir fignum uncertaintyTest
    load(fullfile(testDir, 'overall_trends.mat'));

    % XXX: hard-coded numbers
    ylim1 = 100*[0, 1.1];
    ylim2 = [0, 2];
    if uncertaintyTest
        ylim3 = [0, 2];
        ylim4 = [0, 2];
    else
        ylim3 = [0, 8];
        ylim4 = [0, 5];
    end
    if suboptimal
        ylim5 = [0, 10];
    else
        ylim5 = [0, 5];
    end

    ignore_outliers = true;
    if ignore_outliers
        % Adjust data by counting "blown up" runs (in terms of H-infinity norm and/or
        % integrated cost metrics) as unstable.
        [Stable_H2_baseline, HInfinityNorm_H2_baseline, DiskMargin_H2_baseline,...
            LQCost_H2_baseline, ControlEffort_H2_baseline] = remove_outliers(...
            Stable_H2_baseline, HInfinityNorm_H2_baseline, DiskMargin_H2_baseline,...
            LQCost_H2_baseline, ControlEffort_H2_baseline);
    
        [Stable_H2_indirect, HInfinityNorm_H2_indirect, DiskMargin_H2_indirect,...
            LQCost_H2_indirect, ControlEffort_H2_indirect] = remove_outliers(...
            Stable_H2_indirect, HInfinityNorm_H2_indirect, DiskMargin_H2_indirect,...
            LQCost_H2_indirect, ControlEffort_H2_indirect);
    
        [Stable_H2_direct, HInfinityNorm_H2_direct, DiskMargin_H2_direct,...
            LQCost_H2_direct, ControlEffort_H2_direct] = remove_outliers(...
            Stable_H2_direct, HInfinityNorm_H2_direct, DiskMargin_H2_direct,...
            LQCost_H2_direct, ControlEffort_H2_direct);
    
        [Stable_HInf_baseline, HInfinityNorm_HInf_baseline, DiskMargin_HInf_baseline,...
            LQCost_HInf_baseline, ControlEffort_HInf_baseline] = remove_outliers(...
            Stable_HInf_baseline, HInfinityNorm_HInf_baseline, DiskMargin_HInf_baseline,...
            LQCost_HInf_baseline, ControlEffort_HInf_baseline);
    
        [Stable_HInf_indirect, HInfinityNorm_HInf_indirect, DiskMargin_HInf_indirect,...
            LQCost_HInf_indirect, ControlEffort_HInf_indirect] = remove_outliers(...
            Stable_HInf_indirect, HInfinityNorm_HInf_indirect, DiskMargin_HInf_indirect,...
            LQCost_HInf_indirect, ControlEffort_HInf_indirect);
    
        [Stable_Lyap_direct, HInfinityNorm_Lyap_direct, DiskMargin_Lyap_direct,...
            LQCost_Lyap_direct, ControlEffort_Lyap_direct] = remove_outliers(...
            Stable_Lyap_direct, HInfinityNorm_Lyap_direct, DiskMargin_Lyap_direct,...
            LQCost_Lyap_direct, ControlEffort_Lyap_direct);
    end

    % Instantiate dynamical plant
    if caseStudy == "double-integrator"
        sys = DoubleIntegrator;
        titlestr = "Double Integrator";
        tilenum = 1;
    elseif caseStudy == "cart-pole"
        sys = CartPole;
        titlestr = "Cart Pole";
        tilenum = 2;
    elseif caseStudy == "robot-arm"
        sys = RobotArm;
        titlestr = "Robot Arm";
        tilenum = 3;
    end

    % Percentage of stable learned controllers
    figure(fignum); nexttile(tilenum); cla; hold on; l1 = plot(...
        100*scaleFactor, 100*nanmean(Stable_H2_baseline), 'r--',...
        100*scaleFactor, 100*nanmean(Stable_HInf_baseline), 'k--'...
    );
    b1 = bar(100*scaleFactor, 100*[... 
        nanmean(Stable_H2_indirect);
        nanmean(Stable_H2_direct);
        nanmean(Stable_HInf_indirect);
    ]); hold off; grid on; ylim(ylim1);

    % Disk-based stability margins of output loop transfer function
    figure(fignum); nexttile(tilenum + 3); cla; hold on; l2 = plot(...
        100*scaleFactor, nanmean(DiskMargin_H2_baseline), 'r--',...
        100*scaleFactor, nanmean(DiskMargin_HInf_baseline), 'k--'...
    ); b2 = bar(100*scaleFactor, [...        
        nanmean(DiskMargin_H2_indirect);
        nanmean(DiskMargin_H2_direct);
        nanmean(DiskMargin_HInf_indirect);
    ]); hold off; grid on; %ylim(ylim2);

    % H-infinity norm (peak of singular values of generalized plant)
    gammNormalize = HInfinityNorm_H2_baseline(1,1); % normalize to baseline H2 (LQR)
    figure(fignum); nexttile(tilenum + 6); cla; hold on; l3 = plot(...
        100*scaleFactor, nanmean(HInfinityNorm_H2_baseline)*(1/gammNormalize), 'r--',...
        100*scaleFactor, nanmean(HInfinityNorm_HInf_baseline)*(1/gammNormalize), 'k--'...
    ); b3 = bar(100*scaleFactor, [...        
        nanmean(HInfinityNorm_H2_indirect)*(1/gammNormalize);
        nanmean(HInfinityNorm_H2_direct)*(1/gammNormalize);
        nanmean(HInfinityNorm_HInf_indirect)*(1/gammNormalize);
    ]); hold off; grid on; %ylim(ylim3);

    % Total LQ control cost
    figure(fignum); nexttile(tilenum + 9); cla; hold on; l4 = plot(...
        100*scaleFactor, nanmean(LQCost_H2_baseline), 'r--',...
        100*scaleFactor, nanmean(LQCost_HInf_baseline), 'k--'...
    ); b4 = bar(100*scaleFactor, [...        
        nanmean(LQCost_H2_indirect);
        nanmean(LQCost_H2_direct);
        nanmean(LQCost_HInf_indirect);
    ]); hold off; grid on; %ylim(ylim4);

    % Total control effort
    figure(fignum); nexttile(tilenum + 12); cla; hold on; l5 = plot(...
        100*scaleFactor, nanmean(ControlEffort_H2_baseline), 'r--',...
        100*scaleFactor, nanmean(ControlEffort_HInf_baseline), 'k--'...
    ); b5 = bar(100*scaleFactor, [...        
        nanmean(ControlEffort_H2_indirect);
        nanmean(ControlEffort_H2_direct);
        nanmean(ControlEffort_HInf_indirect);
    ]); hold off; grid on; %ylim(ylim5);

    b1(1).FaceColor = [1, 0, 0]; %b1(1).EdgeColor = b1(1).FaceColor;
    b2(1).FaceColor = [1, 0, 0]; %b2(1).EdgeColor = b2(1).FaceColor;
    b3(1).FaceColor = [1, 0, 0]; %b3(1).EdgeColor = b3(1).FaceColor;
    b4(1).FaceColor = [1, 0, 0]; %b4(1).EdgeColor = b4(1).FaceColor;
    b5(1).FaceColor = [1, 0, 0]; %b5(1).EdgeColor = b5(1).FaceColor;

    b1(2).FaceColor = [0, 0, 1]; %b1(2).EdgeColor = b1(2).FaceColor;
    b2(2).FaceColor = [0, 0, 1]; %b2(2).EdgeColor = b2(2).FaceColor;
    b3(2).FaceColor = [0, 0, 1]; %b3(2).EdgeColor = b3(2).FaceColor;
    b4(2).FaceColor = [0, 0, 1]; %b4(2).EdgeColor = b4(2).FaceColor;
    b5(2).FaceColor = [0, 0, 1]; %b5(2).EdgeColor = b5(2).FaceColor;

    b1(3).FaceColor = [0, 0, 0]; %b1(3).EdgeColor = b1(3).FaceColor;
    b2(3).FaceColor = [0, 0, 0]; %b2(3).EdgeColor = b2(3).FaceColor;
    b3(3).FaceColor = [0, 0, 0]; %b3(3).EdgeColor = b3(3).FaceColor;
    b4(3).FaceColor = [0, 0, 0]; %b4(3).EdgeColor = b4(3).FaceColor;
    b5(3).FaceColor = [0, 0, 0]; %b5(3).EdgeColor = b5(3).FaceColor;

    alpha(b1, 0.65);
    alpha(b2, 0.65);
    alpha(b3, 0.65);
    alpha(b4, 0.65);
    alpha(b5, 0.65);

    %l1(1).LineWidth = 1.25; l1(2).LineWidth = 1.25;
    l2(1).LineWidth = 1.25; l2(2).LineWidth = 1.25;
    l3(1).LineWidth = 1.25; l3(2).LineWidth = 1.25;
    l4(1).LineWidth = 1.25; l4(2).LineWidth = 1.25;
    l5(1).LineWidth = 1.25; l5(2).LineWidth = 1.25;

    figure(fignum); nexttile(tilenum);
    title(titlestr,'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
end

function [] = make_single_plot_line(caseStudy, testDir, fignum, uncertaintyTest)
    clearvars -except testDir fignum uncertaintyTest
    load(fullfile(testDir, 'overall_trends.mat'));

    % XXX: hard-coded numbers
    ylim1 = [0, 100];
    if uncertaintyTest
        ylim2 = [0.5, 1.5];
        ylim3 = [0, 2];
        ylim4 = [0, 2];
        ylim5 = [0, 5];
    else
        ylim2 = [0, 2];
        ylim3 = [0, 8];
        ylim4 = [0, 5];
        ylim5 = [0, 10];
    end

    ignore_outliers = true;
    if ignore_outliers
        % Adjust data by counting "blown up" runs (in terms of H-infinity norm and/or
        % integrated cost metrics) as unstable.
        [Stable_H2_baseline, HInfinityNorm_H2_baseline, DiskMargin_H2_baseline,...
            LQCost_H2_baseline, ControlEffort_H2_baseline] = remove_outliers(...
            Stable_H2_baseline, HInfinityNorm_H2_baseline, DiskMargin_H2_baseline,...
            LQCost_H2_baseline, ControlEffort_H2_baseline);
    
        [Stable_H2_indirect, HInfinityNorm_H2_indirect, DiskMargin_H2_indirect,...
            LQCost_H2_indirect, ControlEffort_H2_indirect] = remove_outliers(...
            Stable_H2_indirect, HInfinityNorm_H2_indirect, DiskMargin_H2_indirect,...
            LQCost_H2_indirect, ControlEffort_H2_indirect);
    
        [Stable_H2_direct, HInfinityNorm_H2_direct, DiskMargin_H2_direct,...
            LQCost_H2_direct, ControlEffort_H2_direct] = remove_outliers(...
            Stable_H2_direct, HInfinityNorm_H2_direct, DiskMargin_H2_direct,...
            LQCost_H2_direct, ControlEffort_H2_direct);
    
        [Stable_HInf_baseline, HInfinityNorm_HInf_baseline, DiskMargin_HInf_baseline,...
            LQCost_HInf_baseline, ControlEffort_HInf_baseline] = remove_outliers(...
            Stable_HInf_baseline, HInfinityNorm_HInf_baseline, DiskMargin_HInf_baseline,...
            LQCost_HInf_baseline, ControlEffort_HInf_baseline);
    
        [Stable_HInf_indirect, HInfinityNorm_HInf_indirect, DiskMargin_HInf_indirect,...
            LQCost_HInf_indirect, ControlEffort_HInf_indirect] = remove_outliers(...
            Stable_HInf_indirect, HInfinityNorm_HInf_indirect, DiskMargin_HInf_indirect,...
            LQCost_HInf_indirect, ControlEffort_HInf_indirect);
    
        [Stable_Lyap_direct, HInfinityNorm_Lyap_direct, DiskMargin_Lyap_direct,...
            LQCost_Lyap_direct, ControlEffort_Lyap_direct] = remove_outliers(...
            Stable_Lyap_direct, HInfinityNorm_Lyap_direct, DiskMargin_Lyap_direct,...
            LQCost_Lyap_direct, ControlEffort_Lyap_direct);
    end

    % Instantiate dynamical plant
    if caseStudy == "double-integrator"
        sys = DoubleIntegrator;
        titlestr = "Double Integrator";
        tilenum = 1;
    elseif caseStudy == "cart-pole"
        sys = CartPole;
        titlestr = "Cart Pole";
        tilenum = 2;
    elseif caseStudy == "robot-arm"
        sys = RobotArm;
        titlestr = "Robot Arm";
        tilenum = 3;
    end

    % Percentage of stable learned controllers
    figure(fignum); nexttile(tilenum); plot(...
        100*scaleFactor, 100*nanmean(Stable_H2_baseline), 'r--',...
        100*scaleFactor, 100*nanmean(Stable_H2_indirect), 'r*-',...
        100*scaleFactor, 100*nanmean(Stable_H2_direct), 'ro-',...
        100*scaleFactor, 100*nanmean(Stable_HInf_baseline), 'k--',...
        100*scaleFactor, 100*nanmean(Stable_HInf_indirect), 'k*-'...
    ); grid on; ylim(ylim1); %title('Percentage of Stable Learned Controllers');

    % Disk-based stability margins of output loop transfer function
    figure(fignum); nexttile(tilenum + 3); cla; plot(...
        100*scaleFactor, nanmean(DiskMargin_H2_baseline), 'r--',...
        100*scaleFactor, nanmean(DiskMargin_H2_indirect), 'r*-',...
        100*scaleFactor, nanmean(DiskMargin_H2_direct), 'ro-',...
        100*scaleFactor, nanmean(DiskMargin_HInf_baseline), 'k--',...
        100*scaleFactor, nanmean(DiskMargin_HInf_indirect), 'k*-'...
    ); grid on; ylim(ylim2); %title('Disk-Based Stability Margins');

    % H-infinity norm (peak of singular values of generalized plant)
    gammNormalize = HInfinityNorm_H2_baseline(1,1); % normalize to baseline H2 (LQR)
    figure(fignum); nexttile(tilenum + 6); cla; plot(...
        100*scaleFactor, (nanmean(HInfinityNorm_H2_baseline)*(1/gammNormalize)), 'r--',...
        100*scaleFactor, (nanmean(HInfinityNorm_H2_indirect)*(1/gammNormalize)), 'r*-',...
        100*scaleFactor, (nanmean(HInfinityNorm_H2_direct)*(1/gammNormalize)), 'ro-',...
        100*scaleFactor, (nanmean(HInfinityNorm_HInf_baseline)*(1/gammNormalize)), 'k--',...
        100*scaleFactor, (nanmean(HInfinityNorm_HInf_indirect)*(1/gammNormalize)), 'k*-'...
    ); grid on; ylim(ylim3); %title('Worst-Case Disturbance Amplification: ||T_{K}||_{\infty}');

    % Total LQ control cost
    figure(fignum); nexttile(tilenum + 9); cla; plot(...
        100*scaleFactor, nanmean(LQCost_H2_baseline), 'r--',...
        100*scaleFactor, nanmean(LQCost_H2_indirect), 'r*-',...
        100*scaleFactor, nanmean(LQCost_H2_direct), 'ro-',...
        100*scaleFactor, nanmean(LQCost_HInf_baseline), 'k--',...
        100*scaleFactor, nanmean(LQCost_HInf_indirect), 'k*-'...
    ); grid on; ylim(ylim4); %title('Total Integrated LQ Control Cost');

    % Total control effort
    figure(fignum); nexttile(tilenum + 12); cla; plot(...
        100*scaleFactor, nanmean(ControlEffort_H2_baseline), 'r--',...
        100*scaleFactor, nanmean(ControlEffort_H2_indirect), 'r*-',...
        100*scaleFactor, nanmean(ControlEffort_H2_direct), 'ro-',...
        100*scaleFactor, nanmean(ControlEffort_HInf_baseline), 'k--',...
        100*scaleFactor, nanmean(ControlEffort_HInf_indirect), 'k*-'...
    ); grid on; ylim(ylim5); %title('Total Integrated Control Effort');

    figure(fignum); nexttile(tilenum);
    title(titlestr,'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
end

function [Stable, HInfinityNorm, DiskMargin, LQCost, ControlEffort] = remove_outliers(...
    Stable, HInfinityNorm, DiskMargin, LQCost, ControlEffort)

    numRuns = size(Stable, 1);
    numSF = size(Stable, 2);

    % Identify "blown up" test runs according to:
    for ii = 1:numSF
        % Outliers in terms of H-infinity norm
        [adjusted, I1] = filloutliers(HInfinityNorm(:,ii), NaN);
        HInfinityNorm(:,ii) = adjusted;
        HInfinityNorm(HInfinityNorm(:,ii) > 10, ii) = NaN;
        Stable(isnan(HInfinityNorm(:,ii)),ii) = 0;

        % Outliers in terms of LQ cost
        [adjusted, I2] = filloutliers(LQCost(:,ii), NaN);
        LQCost(:,ii) = adjusted;
        LQCost(LQCost(:,ii) > 10, ii) = NaN;
        Stable(isnan(LQCost(:,ii)),ii) = 0;

        % Outliers in terms of control effort
        [adjusted, I3] = filloutliers(ControlEffort(:,ii), NaN);
        ControlEffort(:,ii) = adjusted;
        ControlEffort(ControlEffort(:,ii) > 10, ii) = NaN;
        Stable(isnan(ControlEffort(:,ii)),ii) = 0;

        % Adjust corresponding disk margin data
        indsDiskMargin(~Stable(:,ii),ii) = NaN;
    end
end

function normalize_ylims(fignum)
    % Find biggest y-axis limit from each row, apply to each column in the row

    % Row 1
    figure(fignum); nexttile(1); yl1 = ylim;
    figure(fignum); nexttile(2); yl2 = ylim;
    figure(fignum); nexttile(3); yl3 = ylim;
    ylr1 = [min([yl1, yl2, yl3]), max([yl1, yl2, yl3])];
    figure(fignum); nexttile(1); ylim(ylr1);
    figure(fignum); nexttile(2); ylim(ylr1);
    figure(fignum); nexttile(3); ylim(ylr1);

    % Row 2
    figure(fignum); nexttile(4); yl4 = ylim;
    figure(fignum); nexttile(5); yl5 = ylim;
    figure(fignum); nexttile(6); yl6 = ylim;
    ylr2 = [min([yl4, yl5, yl6]), max([yl4, yl5, yl6])];
    figure(fignum); nexttile(4); ylim(ylr2);
    figure(fignum); nexttile(5); ylim(ylr2);
    figure(fignum); nexttile(6); ylim(ylr2);

    % Row 3
    figure(fignum); nexttile(7); yl7 = ylim;
    figure(fignum); nexttile(8); yl8 = ylim;
    figure(fignum); nexttile(9); yl9 = ylim;
    ylr3 = [min([yl7, yl8, yl9]), max([yl7, yl8, yl9])];
    figure(fignum); nexttile(7); ylim(ylr3);
    figure(fignum); nexttile(8); ylim(ylr3);
    figure(fignum); nexttile(9); ylim(ylr3);

    % Row 4
    figure(fignum); nexttile(10); yl10 = ylim;
    figure(fignum); nexttile(11); yl11 = ylim;
    figure(fignum); nexttile(12); yl12 = ylim;
    ylr4 = [min([yl10, yl11, yl12]), max([yl10, yl11, yl12])];
    figure(fignum); nexttile(10); ylim(ylr4);
    figure(fignum); nexttile(11); ylim(ylr4);
    figure(fignum); nexttile(12); ylim(ylr4);

    % Row 5
    figure(fignum); nexttile(13); yl13 = ylim;
    figure(fignum); nexttile(14); yl14 = ylim;
    figure(fignum); nexttile(15); yl15 = ylim;
    ylr5 = [min([yl13, yl14, yl15]), max([yl13, yl14, yl15])];
    figure(fignum); nexttile(13); ylim(ylr5);
    figure(fignum); nexttile(14); ylim(ylr5);
    figure(fignum); nexttile(15); ylim(ylr5);
end