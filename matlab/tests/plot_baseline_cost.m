function [testDir] = plot_baseline_cost(fignum)
    % Total integrated cost metrics
    figure(fignum); clf; tl = tiledlayout(2,3,'TileSpacing','Compact');
    plot_integrated_cost(fignum, "double-integrator");
    plot_integrated_cost(fignum, "cart-pole");
    plot_integrated_cost(fignum, "robot-arm");
    %for ii = 1:tl.GridSize(1)*tl.GridSize(2)
    %    figure(fignum); nexttile(ii);
    %    legend('$\mathcal{H}_{2}$', '$\mathcal{H}_{\infty}$',...
    %        'Interpreter','latex', 'FontSize',14,'Location','SouthEast');
    %end
    figure(fignum); nexttile(1); ylabel('Total Cost','FontName','Helvetica Neue','FontSize',16);
    figure(fignum); nexttile(4); ylabel('Total Effort','FontName','Helvetica Neue','FontSize',16);
    for ii = 4:6
        figure(fignum); nexttile(ii);
        xlabel('Time (s)','FontName','Helvetica Neue','FontSize',16);
    end
    fix_limits(fignum, [0, 1.25], [0, 3]);

%     % Average integrated cost metrics
%     figure(fignum + 1); clf; tl = tiledlayout(2,3,'TileSpacing','Compact');
%     plot_average_cost(fignum + 1, "double-integrator");
%     plot_average_cost(fignum + 1, "cart-pole");
%     plot_average_cost(fignum + 1, "robot-arm");
%     for ii = 1:tl.GridSize(1)*tl.GridSize(2)
%         figure(fignum + 1); nexttile(ii);
%         legend('$\mathcal{H}_{2}$', '$\mathcal{H}_{\infty}$',...
%             'Interpreter','latex','FontSize',14,'Location','NorthEast');
%     end
%     figure(fignum + 1); nexttile(1); ylabel('Average Cost','FontName','Helvetica Neue','FontSize',16);
%     figure(fignum + 1); nexttile(4); ylabel('Average Effort','FontName','Helvetica Neue','FontSize',16);
%     for ii = 4:6
%         figure(fignum + 1); nexttile(ii);
%         xlabel('Time (s)','FontName','Helvetica Neue','FontSize',16);
%    end
end

%% Subroutines
function [] = plot_integrated_cost(fignum, caseStudy)
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
        [J, Ju] = sys.compute_cost(t, u, x, w);
        J_star = J(end); Ju_star = Ju(end);

        figure(fignum); nexttile(tilenum); hold on;
        plot(t, J/J_star, 'r'); grid on; ax = gca;
        ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
        ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';

        figure(fignum); nexttile(tilenum + 3); hold on;
        plot(t, Ju/Ju_star, 'r'); grid on; ax = gca;
        ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
        ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
    end
    disp('  Testing baseline Hinf');
    [stable, ~, ~, ~, ~, ~, ~, ~, ~, t, x, u, y, w, SV] = sys.run_test(K_hinf_b);
    if stable
        [J, Ju] = sys.compute_cost(t, u, x, w);

        figure(fignum); nexttile(tilenum); hold on;
        plot(t, J/J_star, 'k'); grid on; ax = gca;
        ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
        ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';

        figure(fignum); nexttile(tilenum + 3); hold on;
        plot(t, Ju/Ju_star, 'k'); grid on; ax = gca;
        ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
        ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
    end

    figure(fignum); nexttile(tilenum);
    title(titlestr,'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
end

function [] = fix_limits(fignum, ylim1, ylim2)
    ylim12 = [ylim1; ylim2];
    jj = 1;
    for ii = [1, 4]
        figure(fignum); nexttile(ii); ylim(ylim12(jj,:));
        figure(fignum); nexttile(ii); plot(0:30, ones(31,1), 'k--', 'LineWidth', 1); % XXX: hard-coded time vector
    
        figure(fignum); nexttile(ii+1); ylim(ylim12(jj,:));
        figure(fignum); nexttile(ii+1);
        lqLine = plot(0:10, ones(11,1), 'k--'); % XXX: hard-coded time vector
        lqLine.LineWidth = 1;
    
        dummyH2Line = plot(0:10, -1*ones(11,1), 'r'); % XXX: hard-coded time vector
        dummyH2Line.LineWidth = 1;

        dummyHInfLine = plot(0:10, -1*ones(11,1), 'k'); % XXX: hard-coded time vector
        dummyHInfLine.LineWidth = 1;
    
        figure(fignum); nexttile(ii+2); ylim(ylim12(jj,:));
        figure(fignum); nexttile(ii+2); plot(0:20, ones(21,1), 'k--', 'LineWidth', 1); % XXX: hard-coded time vector

        jj = jj + 1;
    end
    
    figure(fignum); nexttile(5); % XXX: hard-coded index
    legend([lqLine, dummyH2Line, dummyHInfLine],...
        'LQR (baseline)', '$\mathcal{H}_{2}$ (baseline)',...
        '$\mathcal{H}_{\infty}$ (baseline)',...
        'Location','SouthOutside','Orientation','horizontal','Interpreter',...
        'latex','FontSize',16);
end

function [] = plot_average_cost(fignum, caseStudy)
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
        [J, Ju] = sys.compute_cost(t, u, x, w);
        J = J./(1:length(J));
        Ju = Ju./(1:length(Ju));

        figure(fignum); nexttile(tilenum); hold on;
        semilogy(t, J, 'r'); grid on; ax = gca;
        ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
        ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';

        figure(fignum); nexttile(tilenum + 3); hold on;
        semilogy(t, Ju, 'r'); grid on; ax = gca;
        ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
        ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
    end
    disp('  Testing baseline Hinf');
    [stable, ~, ~, ~, ~, ~, ~, ~, ~, t, x, u, y, w, SV] = sys.run_test(K_hinf_b);
    if stable
        [J, Ju] = sys.compute_cost(t, u, x, w);
        J = J./(1:length(J));
        Ju = Ju./(1:length(Ju));

        figure(fignum); nexttile(tilenum); hold on;
        semilogy(t, J, 'k'); grid on; ax = gca;
        ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
        ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';

        figure(fignum); nexttile(tilenum + 3); hold on;
        semilogy(t, Ju, 'k'); grid on; ax = gca;
        ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
        ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
    end

    figure(fignum); nexttile(tilenum);
    title(titlestr,'FontName','Helvetica Neue','FontSize',16,'FontWeight','Bold');
end
