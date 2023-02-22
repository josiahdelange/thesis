function [] = run_all_integrated_metrics(fignums)
    %% Runs all the uncertainty and noise tests to get integrated cost metrics
    outputDir = '/Users/josiah/ms-thesis/latex/figures';

    %% Canonical/optimal LMI formulation for H2 direct
    tic;

    % Integrated control effort: parameter uncertainty
    disp('--> Running scaled uncertainty test (H2 optimal)');
    run_uncertainty_tests(fignums(1), 100, false, false, true);
    fix_ylimits(fignums(1), [0, 3], [0, 1.25], [0, 1.5], false);

    % Integrated LQ cost: parameter uncertainty
    disp('--> Running scaled uncertainty test (H2 optimal)');
    run_uncertainty_tests(fignums(2), 100, false, false, false);
    fix_ylimits(fignums(2), [0, 1.25], [0, 1.25], [0, 1.25], false);

    % Integrated control effort: measurement noise
    disp('--> Running scaled noise test (H2 optimal)');
    run_noise_tests(fignums(3), 100, false, false, false);
    fix_ylimits(fignums(3), [0, 2], [0, 2], [0, 2], false);

    % Integrated LQ cost: measurement noise
    disp('--> Running scaled noise test (H2 optimal)');
    run_noise_tests(fignums(4), 100, false, false, true);
    fix_ylimits(fignums(4), [0, 3], [0, 2], [0, 2], false);

    toc;

    %% Adjusted/suboptimal LMI formulation for H2 direct
    tic;

    % Integrated control effort: parameter uncertainty
    disp('--> Running scaled uncertainty test (H2 suboptimal)');
    run_uncertainty_tests(fignums(7), 100, true, false, true);
    fix_ylimits(fignums(7), [0, 10], [0, 2], [0, 2], true);

    % Integrated LQ cost: parameter uncertainty
    disp('--> Running scaled uncertainty test (H2 suboptimal)');
    run_uncertainty_tests(fignums(5), 100, true, false, false);
    fix_ylimits(fignums(5), [0, 3], [0, 1.5], [0, 2], true);

    % Integrated control effort: measurement noise
    disp('--> Running scaled noise test (H2 suboptimal)');
    run_noise_tests(fignums(6), 100, true, false, false);
    fix_ylimits(fignums(6), [0, 3], [0, 1.5], [0, 2], true);

    % Integrated LQ cost: measurement noise
    disp('--> Running scaled noise test (H2 suboptimal)');
    run_noise_tests(fignums(8), 100, true, false, true);
    fix_ylimits(fignums(8), [0, 10], [0, 2], [0, 2], true);

    toc;

    %% Save output files to disk
    % Save .fig files
    disp('Saving .fig files...');
    savefig(fignums(2), fullfile(outputDir, 'uncertainty_integrated_cost3'));
    savefig(fignums(3), fullfile(outputDir, 'noise_integrated_cost3'));
    savefig(fignums(1), fullfile(outputDir, 'uncertainty_integrated_effort3'));
    savefig(fignums(4), fullfile(outputDir, 'noise_integrated_effort3'));
    
    savefig(fignums(5), fullfile(outputDir, 'uncertainty_integrated_cost4_s'));
    savefig(fignums(6), fullfile(outputDir, 'noise_integrated_cost4_s'));
    savefig(fignums(7), fullfile(outputDir, 'uncertainty_integrated_effort4_s'));
    savefig(fignums(8), fullfile(outputDir, 'noise_integrated_effort4_s'));
    disp('...done');
    
    % Save .png files
    disp('Saving .png files...');
    saveas(fignums(2), fullfile(outputDir, 'uncertainty_integrated_cost3.png'));
    saveas(fignums(3), fullfile(outputDir, 'noise_integrated_cost3.png'));
    saveas(fignums(1), fullfile(outputDir, 'uncertainty_integrated_effort3.png'));
    saveas(fignums(4), fullfile(outputDir, 'noise_integrated_effort3.png'));
    
    saveas(fignums(5), fullfile(outputDir, 'uncertainty_integrated_cost4_s.png'));
    saveas(fignums(6), fullfile(outputDir, 'noise_integrated_cost4_s.png'));
    saveas(fignums(7), fullfile(outputDir, 'uncertainty_integrated_effort4_s.png'));
    saveas(fignums(8), fullfile(outputDir, 'noise_integrated_effort4_s.png'));
    disp('...done');
end

%% Subroutines
function [lh1, lh2, lh3] = test_and_plot_uncertainty(fignum, tilenum, caseStudy,...
    numRuns, scaleFactor, suboptimal, average, effort, normalizeLQR)

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

    if nargin < 9
        normalizeLQR = true;
    end

    J_h2_i = nan(numRuns, sys.N-1);
    J_h2_d = nan(numRuns, sys.N-1);
    J_hinf_i = nan(numRuns, sys.N-1);

    % Minimax/optimal gamma
    gamma_star = minimaxPolicy(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, eps, 1e5, 1.5);
    sys.ctrl.gamma = gamma_star;

    % LQR optimal cost and control effort
    K_h2_b = BaselineMethod(sys, Inf);
    [t, x, u, ~, ~, w] = sys.simulate(sys.x0, sys.u0, sys.N, @(t,x) K_h2_b*x);
    [J_star, Ju_star] = sys.compute_cost(t, u, x, w);
    J_star = J_star(end); Ju_star = Ju_star(end);
    if effort
        J_star_ = Ju_star;
    else
        J_star_ = J_star;
    end

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
        K_hinf_i = IndirectMethod(sys, gamma_star, y_t, u_t); % H-inf (indirect)

        % Evaluate results (indirect H2)
        if check_stability(sys.sysd.A, sys.sysd.B, K_h2_i)
            % Simulate the plant with closed-loop feedback and some initial conditions
            [t, x, u, y, delta, w] = sys.simulate(sys.x0, sys.u0, sys.N, @(t,x) K_h2_i*x);

            % Compare regulator cost and control effort
            [J, Ju, Jg] = sys.compute_cost(t, u, x, w);
            if effort
                J_ = Ju;
            else
                J_ = J;
            end
            if normalizeLQR
                J_ = J_/J_star_;
            end
            if average
                J_ = J_./(1:length(J_));
            end
            J_h2_i(n,:) = J_;
        end

        % Evaluate results (direct H2)
        if check_stability(sys.sysd.A, sys.sysd.B, K_h2_d)
            % Simulate the plant with closed-loop feedback and some initial conditions
            [t, x, u, y, delta, w] = sys.simulate(sys.x0, sys.u0, sys.N, @(t,x) K_h2_d*x);

            % Compare regulator cost and control effort
            [J, Ju, Jg] = sys.compute_cost(t, u, x, w);
            if effort
                J_ = Ju;
            else
                J_ = J;
            end
            if normalizeLQR
                J_ = J_/J_star_;
            end
            if average
                J_ = J_./(1:length(J_));
            end
            J_h2_d(n,:) = J_;
        end

        % Evaluate results (indirect H-infinity)
        if check_stability(sys.sysd.A, sys.sysd.B, K_hinf_i)
            % Simulate the plant with closed-loop feedback and some initial conditions
            [t, x, u, y, delta, w] = sys.simulate(sys.x0, sys.u0, sys.N, @(t,x) K_hinf_i*x);

            % Compare regulator cost and control effort
            [J, Ju, Jg] = sys.compute_cost(t, u, x, w);
            if effort
                J_ = Ju;
            else
                J_ = J;
            end
            if normalizeLQR
                J_ = J_/J_star_;
            end
            if average
                J_ = J_./(1:length(J_));
            end
            J_hinf_i(n,:) = J_;
        end
    end

    figure(fignum); nexttile(tilenum); hold on;
    lh3 = plot(t, J_hinf_i, 'k');
    for l = 1:size(lh3)
        lh3(l).Color(4) = 0.7;
    end
    lh1 = plot(t, J_h2_i, 'r');
    for l = 1:size(lh1)
        lh1(l).Color(4) = 0.7;
    end
    lh2 = plot(t, J_h2_d, 'b');
    for l = 1:size(lh2)
        lh2(l).Color(4) = 0.7;
    end
    grid on;
end

function [] = run_uncertainty_tests(fignum, numRuns, suboptimal, average, effort)
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
        test_and_plot_uncertainty(fignum, ii, "double-integrator", numRuns, SF, suboptimal, average, effort);
        figure(fignum); nexttile(ii);
        word1 = '';
        if average
            word1 = 'Average';
        else
            word1 = 'Total';
        end
        word2 = '';
        if effort
            word2 = 'Effort';
        else
            word2 = 'Cost';
        end
        ylabel([word1 ' ' word2],'FontName','Helvetica Neue','FontSize',16);
        [lh1, lh2, lh3] = test_and_plot_uncertainty(fignum, ii + 1, "cart-pole", numRuns, SF, suboptimal, average, effort);
        test_and_plot_uncertainty(fignum, ii + 2, "robot-arm", numRuns, SF, suboptimal, average, effort);
        ii = ii + 3;
    end

    ii = ii - 3;
    for jj = 0:2
        figure(fignum); nexttile(ii + jj);
        xlabel('Time (s)','FontName','Helvetica Neue','FontSize',16);
    end
    figure(fignum); nexttile(ii + 1); ax = gca;
    if suboptimal
        legend([lh1(1), lh2(1), lh3(1)], '$\mathcal{H}_{2}$ (indirect)',...
            '$\mathcal{H}_{2}$ (direct, suboptimal)', '$\mathcal{H}_{\infty}$ (indirect)',...
            'Location','SouthOutside','Orientation','horizontal','Interpreter',...
            'latex','FontSize',16);
    else
        legend([lh1(1), lh2(1), lh3(1)], '$\mathcal{H}_{2}$ (indirect)',...
            '$\mathcal{H}_{2}$ (direct)', '$\mathcal{H}_{\infty}$ (indirect)',...
            'Location','SouthOutside','Orientation','horizontal','Interpreter',...
            'latex','FontSize',16);
    end

    figure(fignum); nexttile(1); title("Double Integrator",...
        'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
    figure(fignum); nexttile(2); title("Cart Pole",...
        'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
    figure(fignum); nexttile(3); title("Robot Arm",...
        'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
end

function [lh1, lh2, lh3] = test_and_plot_noise(fignum, tilenum, caseStudy,...
    numRuns, scaleFactor, suboptimal, average, effort, normalizeLQR)

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

    if nargin < 9
        normalizeLQR = true;
    end

    J_h2_i = nan(numRuns, sys.N-1);
    J_h2_d = nan(numRuns, sys.N-1);
    J_hinf_i = nan(numRuns, sys.N-1);

    % Minimax/optimal gamma
    gamma_star = minimaxPolicy(sys.sysd.A, sys.sysd.B, sys.ctrl.D, sys.ctrl.H, sys.ctrl.G, eps, 1e5, 1.5);
    sys.ctrl.gamma = gamma_star;

    % LQR optimal cost and control effort
    K_h2_b = BaselineMethod(sys, Inf);
    [t, x, u, ~, ~, w] = sys.simulate(sys.x0, sys.u0, sys.N, @(t,x) K_h2_b*x);
    [J_star, Ju_star] = sys.compute_cost(t, u, x, w);
    J_star = J_star(end); Ju_star = Ju_star(end);
    if effort
        J_star_ = Ju_star;
    else
        J_star_ = J_star;
    end

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
        K_hinf_i = IndirectMethod(sys, gamma_star, y_t, u_t); % H-inf (indirect)

        % Evaluate results (indirect H2)
        if check_stability(sys.sysd.A, sys.sysd.B, K_h2_i)
            % Simulate the plant with closed-loop feedback and some initial conditions
            [t, x, u, y, delta, w] = sys.simulate(sys.x0, sys.u0, sys.N, @(t,x) K_h2_i*x);

            % Compare regulator cost and control effort
            [J, Ju, Jg] = sys.compute_cost(t, u, x, w);
            if effort
                J_ = Ju;
            else
                J_ = J;
            end
            if normalizeLQR
                J_ = J_/J_star_;
            end
            if average
                J_ = J_./(1:length(J_));
            end
            J_h2_i(n,:) = J_;
        end

        % Evaluate results (direct H2)
        if check_stability(sys.sysd.A, sys.sysd.B, K_h2_d)
            % Simulate the plant with closed-loop feedback and some initial conditions
            [t, x, u, y, delta, w] = sys.simulate(sys.x0, sys.u0, sys.N, @(t,x) K_h2_d*x);

            % Compare regulator cost and control effort
            [J, Ju, Jg] = sys.compute_cost(t, u, x, w);
            if effort
                J_ = Ju;
            else
                J_ = J;
            end
            if normalizeLQR
                J_ = J_/J_star_;
            end
            if average
                J_ = J_./(1:length(J_));
            end
            J_h2_d(n,:) = J_;
        end

        % Evaluate results (indirect H-infinity)
        if check_stability(sys.sysd.A, sys.sysd.B, K_hinf_i)
            % Simulate the plant with closed-loop feedback and some initial conditions
            [t, x, u, y, delta, w] = sys.simulate(sys.x0, sys.u0, sys.N, @(t,x) K_hinf_i*x);

            % Compare regulator cost and control effort
            [J, Ju, Jg] = sys.compute_cost(t, u, x, w);
            if effort
                J_ = Ju;
            else
                J_ = J;
            end
            if normalizeLQR
                J_ = J_/J_star_;
            end
            if average
                J_ = J_./(1:length(J_));
            end
            J_hinf_i(n,:) = J_;
        end
    end

    figure(fignum); nexttile(tilenum); hold on;
    lh3 = plot(t, J_hinf_i, 'k');
    for l = 1:size(lh3)
        lh3(l).Color(4) = 0.7;
    end
    lh1 = plot(t, J_h2_i, 'r');
    for l = 1:size(lh1)
        lh1(l).Color(4) = 0.7;
    end
    lh2 = plot(t, J_h2_d, 'b');
    for l = 1:size(lh2)
        lh2(l).Color(4) = 0.7;
    end
    grid on;
end

function [] = run_noise_tests(fignum, numRuns, suboptimal, average, effort)
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
        test_and_plot_noise(fignum, ii, "double-integrator", numRuns, SF, suboptimal, average, effort);
        figure(fignum); nexttile(ii);
        word1 = '';
        if average
            word1 = 'Average';
        else
            word1 = 'Total';
        end
        word2 = '';
        if effort
            word2 = 'Effort';
        else
            word2 = 'Cost';
        end
        ylabel([word1 ' ' word2],'FontName','Helvetica Neue','FontSize',16);
        [lh1, lh2, lh3] = test_and_plot_noise(fignum, ii + 1, "cart-pole", numRuns, SF, suboptimal, average, effort);
        test_and_plot_noise(fignum, ii + 2, "robot-arm", numRuns, SF, suboptimal, average, effort);
        ii = ii + 3;
    end

    ii = ii - 3;
    for jj = 0:2
        figure(fignum); nexttile(ii + jj);
        xlabel('Time (s)','FontName','Helvetica Neue','FontSize',16);
    end
    figure(fignum); nexttile(ii + 1); ax = gca;
    if suboptimal
        legend([lh1(1), lh2(1), lh3(1)], '$\mathcal{H}_{2}$ (indirect)',...
            '$\mathcal{H}_{2}$ (direct, suboptimal)', '$\mathcal{H}_{\infty}$ (indirect)',...
            'Location','SouthOutside','Orientation','horizontal','Interpreter',...
            'latex','FontSize',16);
    else
        legend([lh1(1), lh2(1), lh3(1)], '$\mathcal{H}_{2}$ (indirect)',...
            '$\mathcal{H}_{2}$ (direct)', '$\mathcal{H}_{\infty}$ (indirect)',...
            'Location','SouthOutside','Orientation','horizontal','Interpreter',...
            'latex','FontSize',16);
    end

    figure(fignum); nexttile(1); title("Double Integrator",...
        'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
    figure(fignum); nexttile(2); title("Cart Pole",...
        'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
    figure(fignum); nexttile(3); title("Robot Arm",...
        'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
end

function [] = fix_ylimits(fignum, ylim1, ylim2, ylim3, suboptimal)
    for ii = [1, 4, 7, 10]
        figure(fignum); nexttile(ii); ylim(ylim1);
        figure(fignum); nexttile(ii); plot(0:30, ones(31,1), 'k--', 'LineWidth', 1); % XXX: hard-coded time vector
    
        figure(fignum); nexttile(ii+1); ylim(ylim2);
        figure(fignum); nexttile(ii+1);
        lqLine = plot(0:10, ones(11,1), 'k--'); % XXX: hard-coded time vector
        lqLine.LineWidth = 1;
    
        dummyH2IndirectLine = plot(0:10, -1*ones(11,1), 'r'); % XXX: hard-coded time vector
        dummyH2IndirectLine.LineWidth = 1;
    
        dummyH2DirectLine = plot(0:10, -1*ones(11,1), 'b'); % XXX: hard-coded time vector
        dummyH2DirectLine.LineWidth = 1;
    
        dummyHInfIndirectLine = plot(0:10, -1*ones(11,1), 'k'); % XXX: hard-coded time vector
        dummyHInfIndirectLine.LineWidth = 1;
    
        figure(fignum); nexttile(ii+2); ylim(ylim3);
        figure(fignum); nexttile(ii+2); plot(0:fignums(2), ones(fignums(3),1), 'k--', 'LineWidth', 1); % XXX: hard-coded time vector
    end
    
    figure(fignum); nexttile(11); % XXX: hard-coded index

    if suboptimal
        legend([lqLine, dummyH2IndirectLine, dummyH2DirectLine, dummyHInfIndirectLine],...
            'LQR (baseline)', '$\mathcal{H}_{2}$ (indirect)',...
            '$\mathcal{H}_{2}$ (direct, suboptimal)', '$\mathcal{H}_{\infty}$ (indirect)',...
            'Location','SouthOutside','Orientation','horizontal','Interpreter',...
            'latex','FontSize',16);
    else
        legend([lqLine, dummyH2IndirectLine, dummyH2DirectLine, dummyHInfIndirectLine],...
            'LQR (baseline)', '$\mathcal{H}_{2}$ (indirect)',...
            '$\mathcal{H}_{2}$ (direct)', '$\mathcal{H}_{\infty}$ (indirect)',...
            'Location','SouthOutside','Orientation','horizontal','Interpreter',...
            'latex','FontSize',16);
    end
end