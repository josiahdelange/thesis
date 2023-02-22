classdef CartPole < DynamicalSystem
    properties
        % Plant model parameters
        m_c {mustBeNumeric}; % cart mass (kg)
        m_p {mustBeNumeric}; % pole mass (kg)
        l0 {mustBeNumeric}; % nominal pole length (m)
        l {mustBeNumeric}; % pole length (m)
        g {mustBeNumeric}; % gravitational acceleration (m/s^2)
    end

    methods
        function obj = CartPole
            % Dynamical plant parameters
            obj = obj@DynamicalSystem;
            obj.name_str = 'cart pole';
            obj.nx = 4;
            obj.nu = 1;
            obj.ny = 4;
            obj.umin = -Inf*ones(obj.nu, 1);
            obj.umax = Inf*ones(obj.nu, 1);

            % Default inertial parameters
            %obj.m_c = 3.25;%1.3282; % mass of cart, kg
            %obj.m_p = 0.1;%0.82; % mass of pole, kg
            %obj.g = -9.81; % gravitational acceleration, m/s^2
            %obj.l = 1;%0.304; % length of pole, m
            obj.m_c = 1.3282; % mass of cart, kg
            obj.m_p = 0.82; % mass of pole, kg
            obj.g = -9.81; % gravitational acceleration, m/s^2
            obj.l0 = 0.304; % nominal length of pole, m
            obj.l = obj.l0; % length of pole, m

            % Default measurement noise parameters
            obj.mu_x = zeros(obj.nx, 1);
            obj.sig2_x = zeros(obj.nx, 1);

            % Training policy parameters
            obj.Nt = 100;
            obj.K0 = [-2, 3, -3, 7];
            obj.mu_exploration = @(t, x) 10.1*exp(-0.8*t).*sin(1*t) + 0.0*randn(obj.nu, 1);

            % Control parameters
            obj.dt = 0.05;
            obj.ctrl.d = [0.1; 0.1*pi/180; 0.05; 0.01*pi/180];
            obj.ctrl.Qx = diag([100; 1000; 10; 1000]);
            obj.ctrl.Ru = 1;

            % Simulation parameters
            obj.N = 200;
            obj.x0 = [0.2; 2*pi/180; 0; 0];
            obj.u0 = 0;

            % Finish initialization
            obj = obj.linearize([0; 0], [0; 0]);
            obj = obj.discretize(obj.dt);
            obj = obj.set_weights;
            obj.odeSolver = "ode45";
        end

        % Compute the control input mapping B_tau
        function [B_] = B_tau(obj, q, q_dot)
            B_ = [1; 0];
        end

        % Compute the mass matrix M(q)
        function [M_q] = M(obj, q)
            M_q(1,1) = obj.m_c + obj.m_p;
            M_q(1,2) = obj.m_p*obj.l*cos(q(2));
            M_q(2,1) = M_q(1,2);
            M_q(2,2) = obj.m_p*obj.l^2;
        end

        % Compute the inverse of the mass matrix
        function [inv_M_q] = invM(obj, q)
            % Get M(q)
            M_q = obj.M(q);

            % Compute det(M(q))
            det_M_q = (M_q(1,1)*M_q(2,2) - M_q(2,1)*M_q(1,2));

            % Compute inv(M(q))
            inv_M_q(1,1) = M_q(2,2)/det_M_q;
            inv_M_q(1,2) = -1*M_q(1,2)/det_M_q;
            inv_M_q(2,1) = -1*M_q(2,1)/det_M_q;
            inv_M_q(2,2) = M_q(1,1)/det_M_q;
        end

        % Compute the Coriolis matrix C(q, qdot)
        function [C_q_qdot] = C(obj, q, q_dot)
            C_q_qdot(1,1) = 0;
            C_q_qdot(1,2) = -obj.m_p*obj.l*q_dot(2)*sin(q(2));
            C_q_qdot(2,1) = 0;
            C_q_qdot(2,2) = 0;
        end

        % Compute the gravitational matrix g(q)
        function [g_q] = G(obj, q)
            g_q(1,1) = 0;
            g_q(2,1) = obj.m_p*obj.g*obj.l*sin(q(2));
        end

        % Compute the partial of g(q) w.r.t. q, for qdot = 0
        function [dgdq] = dG_dq(obj, q, q_dot)
            dgdq(1,1) = 0;
            dgdq(1,2) = 0;
            dgdq(2,1) = 0;
            dgdq(2,2) = obj.m_p*obj.g*obj.l;
        end

        % Continuous-time dynamic model of a cart pole
        function dx = f_ode(obj, t, x, u)
            % Map state vector into joint coordinates
            q = [x(1); x(2)]; % position and angle
            q_dot = [x(3); x(4)]; % velocity and angular rate

            % Map controlled inputs to joint coordinates
            tau = obj.B_tau(q, q_dot)*u;

            % Compute accelerations via the manipulator equation
            qddot = obj.f_manipulator(q, q_dot, tau);

            % Construct state derivatives
            dx = [q_dot; qddot];
        end

        % Set variance of parameter uncertainty
        function [obj] = parameter_uncertainty(obj, scaleFactor)
            obj.l = obj.l0 + obj.l0*scaleFactor*randn;
            obj = obj.linearize([0; 0], [0; 0]);
            obj = obj.discretize(obj.dt);
            obj = obj.set_weights;
        end

        function [] = add_plot_transient_response(obj, fignum, t, u, y, plotColor, makeTitles)
            if nargin < 7
                makeTitles = true;
            end

            figure(fignum); nexttile(1, [1,2]); hold on;
            plot(t, u(1,:), 'Color', plotColor, 'LineWidth', 0.7); grid on; hold on;
            xlabel('Time (s)'); ylabel('$f$ (N-m)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Input Force','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end

            figure(fignum); nexttile(3); hold on;
            plot(t, y(1,:), 'Color', plotColor, 'LineWidth', 0.7); grid on;
            %xlabel('Time (s)');
            ylabel('$y$ (m)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Cart Position','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end

            figure(fignum); nexttile(4); hold on;
            plot(t, 180/pi*y(2,:), 'Color', plotColor, 'LineWidth', 0.7); grid on;
            %xlabel('Time (s)');
            ylabel('$\theta$ (deg)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Pole Angle','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end

            figure(fignum); nexttile(5); hold on;
            plot(t, y(3,:), 'Color', plotColor, 'LineWidth', 0.7); grid on;
            %xlabel('Time (s)');
            ylabel('$\dot{y}$ (m/s)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Cart Velocity','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end

            figure(fignum); nexttile(6); hold on;
            plot(t, 180/pi*y(4,:), 'Color', plotColor, 'LineWidth', 0.7); grid on;
            %xlabel('Time (s)');
            ylabel('$\dot{\theta}$ (deg/s)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Pole Angular Rate','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end
        end
    end
end