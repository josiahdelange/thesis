classdef DoubleIntegrator < DynamicalSystem
    properties
        % Plant model parameters
        m_c {mustBeNumeric}; % cart mass (kg)
        m_c0 {mustBeNumeric}; % nominal cart mass (kg)
    end

    methods
        function obj = DoubleIntegrator
            % Dynamical plant parameters
            obj = obj@DynamicalSystem;
            obj.name_str = 'double integrator';
            obj.nx = 2;
            obj.nu = 1;
            obj.ny = 2;
            obj.umin = -Inf*ones(obj.nu, 1);
            obj.umax = Inf*ones(obj.nu, 1);

            % Default inertial parameters
            obj.m_c0 = 1;
            obj.m_c = obj.m_c0;

            % Default measurement noise parameters
            obj.mu_x = zeros(obj.nx, 1);
            obj.sig2_x = zeros(obj.nx, 1);

            % Training policy parameters
            obj.Nt = 80;
            obj.K0 = [-1.91, -1.01];
            obj.mu_exploration = @(t,x) 0.1*exp(-0.8*t).*sin(4*t) + 0.5*randn(obj.nu, 1);

            % Control parameters
            obj.dt = 0.1;
            obj.ctrl.d = [0.1; 0.05];
            obj.ctrl.Qx = diag([1; 5]);
            obj.ctrl.Ru = 1;

            % Simulation parameters
            obj.N = 300;
            obj.x0 = [1; 0];
            obj.u0 = 0;

            % Finish initialization
            obj = obj.linearize(0, 0);
            obj = obj.discretize(obj.dt);
            obj = obj.set_weights;
            obj.odeSolver = "ode45";
        end

        % Compute the control input mapping B_tau
        function [B_] = B_tau(obj, q, q_dot)
            B_ = 1;
        end

        % Compute the mass matrix M(q)
        function [M_] = M(obj, q)
            M_ = obj.m_c;
        end

        % Compute the inverse of the mass matrix
        function [invM_] = invM(obj, q)
            invM_ = 1/obj.m_c;
        end

        % Compute the Coriolis matrix C(q, qdot)
        function [C_] = C(obj, q, q_dot)
            C_ = 0;
        end

        % Compute the gravitational matrix g(q)
        function [g_] = G(obj, q)
            g_ = 0;
        end

        % Compute the partial of g(q) w.r.t. q, for qdot = 0
        function [dgdq] = dG_dq(obj, q, q_dot)
            dgdq = 0;
        end

        % Continuous-time dynamic model of a double integrator
        function dx = f_ode(obj, t, x, u)
            % Map state vector into joint coordinates
            q = x(1); % position
            q_dot = x(2); % velocity

            % Map controlled inputs to joint coordinates
            tau = obj.B_tau(q, q_dot)*u;

            % Compute accelerations via the manipulator equation
            q_ddot = obj.f_manipulator(q, q_dot, tau);

            % Construct state derivatives
            dx = [q_dot; q_ddot];
        end

        % Set variance of parameter uncertainty
        function [obj] = parameter_uncertainty(obj, scaleFactor)
            obj.m_c = obj.m_c0 + obj.m_c0*scaleFactor*randn;
            obj = obj.linearize(0, 0);
            obj = obj.discretize(obj.dt);
            obj = obj.set_weights;
        end

        function [] = add_plot_transient_response(obj, fignum, t, u, y, plotColor, makeTitles)
            if nargin < 7
                makeTitles = true;
            end

            figure(fignum); nexttile(1); hold on;
            plot(t, u(1,:), 'Color', plotColor, 'LineWidth', 0.7); grid on; hold on;
            %xlabel('Time (s)');
            ylabel('$f$ (N-m)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Input Force','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end

            figure(fignum); nexttile(2); hold on;
            plot(t, y(1,:), 'Color', plotColor, 'LineWidth', 0.7); grid on;
            %xlabel('Time (s)');
            ylabel('$y$ (m)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Cart Position','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end

            figure(fignum); nexttile(3); hold on;
            plot(t, y(2,:), 'Color', plotColor, 'LineWidth', 0.7); grid on;
            xlabel('Time (s)'); ylabel('$\dot{y}$ (m/s)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Cart Velocity','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end
        end
    end
end