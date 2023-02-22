classdef RobotArm < DynamicalSystem
    properties
        % Plant model parameters
        m1 {mustBeNumeric}; % link 1 mass (kg)
        l1 {mustBeNumeric}; % link 1 length (m)
        l1_0 {mustBeNumeric}; % nominal link 1 length (m)
        m2 {mustBeNumeric}; % link 2 mass (kg)
        l2 {mustBeNumeric}; % link 2 length (m)
        g {mustBeNumeric}; % gravitational acceleration (m/s^2)
        beta1 {mustBeNumeric}; % damping of link 1
        beta2 {mustBeNumeric}; % damping of link 2
    end

    methods
        function obj = RobotArm
            % Dynamical plant parameters
            obj = obj@DynamicalSystem;
            obj.name_str = 'robot arm';
            obj.nx = 4;
            obj.nu = 2;
            obj.ny = 4;
            obj.umin = -Inf*ones(obj.nu, 1);
            obj.umax = Inf*ones(obj.nu, 1);

            % Default inertial parameters
            obj.m1 = 4; %.82; % kg
            obj.l1_0 = 5; % m
            obj.l1 = obj.l1_0; %.304; % m
            obj.m2 = 2.5; %.41; % kg
            obj.l2 = 5; %.152; % m
            obj.g = 0; % m/s^2
            obj.beta1 = 0;%-0.01;
            obj.beta2 = 0;%-0.01;

            % Default measurement noise parameters
            obj.mu_x = zeros(obj.nx, 1);
            obj.sig2_x = zeros(obj.nx, 1);

            % Training policy parameters
            obj.Nt = 150;
            obj.K0 = [-60, -3, -200, -60; -3, -60, -60, -68];
            obj.mu_exploration = @(t,x) [...
                5*(2*exp(-0.8*t).*sin(t) + 0.01*randn(1,1));
                5*(2*exp(-0.8*t).*sin(t) + 0.01*randn(1,1));
            ];

            % Control parameters
            obj.dt = 0.1;
            obj.ctrl.d = [0.1*pi/180; 0.1*pi/180; 0.01*pi/180; 0.01*pi/180];
            obj.ctrl.Qx = diag([5000; 5000; 500; 500]);
            obj.ctrl.Ru = diag([1; 1]);

            % Simulation parameters
            obj.N = 200;
            obj.x0 = [3*pi/180; -2*pi/180; 0; 0];
            obj.u0 = [0; 0];

            % Finish initialization
            obj = obj.linearize([0; 0], [0; 0]);
            obj = obj.discretize(obj.dt);
            obj = obj.set_weights;
            obj.odeSolver = "ode15s";
        end

        % Compute the control input mapping B_tau
        function [B_] = B_tau(obj, q, q_dot)
            B_ = eye(2);
        end

        % Compute the mass matrix M(q)
        function [M_q] = M(obj, q)
            M_q(1,1) = (obj.m1 + obj.m2)*obj.l1^2 + obj.m2*obj.l2^2 + 2*obj.m2*obj.l1*obj.l2*cos(q(2));
            M_q(1,2) = obj.m2*obj.l2^2 + obj.m2*obj.l1*obj.l2*cos(q(2));
            M_q(2,1) = M_q(1,2);
            M_q(2,2) = obj.m2*obj.l2^2;
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
        function [C_qqd] = C(obj, q, q_dot)
            C_qqd(1,1) = -2*obj.m2*obj.l1*obj.l2*sin(q(2));
            C_qqd(1,2) = -obj.m2*obj.l1*obj.l2*q_dot(2)*sin(q(2)) + obj.beta1;
            C_qqd(2,1) = obj.m2*obj.l1*obj.l2*q_dot(1)*sin(q(2));
            C_qqd(2,2) = 0 + obj.beta2;
        end

        % Compute the gravitational matrix g(q)
        function [g_q] = G(obj, q)
            g_q(1,1) = (obj.m1 + obj.m2)*obj.g*obj.l1*cos(q(1)) + obj.m2*obj.g*obj.l2*sin(q(1) + q(2));
            g_q(2,1) = obj.m2*obj.g*obj.l2*sin(q(1) + q(2));
        end

        % Compute the partial of g(q) w.r.t. q, for qdot = 0
        function [dgdq] = dG_dq(obj, q, qdot)
            dgdq(1,1) = (obj.m1 + obj.m2)*obj.g*obj.l1*q(1);
            dgdq(1,2) = obj.m2*obj.g*obj.l2*cos(q(1) + q(2));
            dgdq(2,1) = obj.m2*obj.g*obj.l2*cos(q(1) + q(2));
            dgdq(2,2) = obj.m2*obj.g*obj.l2*cos(q(1) + q(2));
        end

        % Continuous-time dynamic model of a robot arm
        function dx = f_ode(obj, t, x, u)
            % Map state vector into joint coordinates
            q = [x(1); x(2)]; % angular displacements
            q_dot = [x(3); x(4)]; % angular velocities

            % Map controlled inputs to joint coordinates
            tau = obj.B_tau(q, q_dot)*u;

            % Compute accelerations via the manipulator equation
            qddot = obj.f_manipulator(q, q_dot, tau);

            % Construct state derivatives
            dx = [q_dot; qddot];
        end

        % Set variance of parameter uncertainty
        function [obj] = parameter_uncertainty(obj, scaleFactor)
            obj.l1 = obj.l1_0 + obj.l1_0*scaleFactor*randn;
            obj = obj.linearize([0; 0], [0; 0]);
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
            ylabel('$\tau_{1}$ (N-m)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Joint 1 Torque','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end

            figure(fignum); nexttile(2); hold on;
            plot(t, u(2,:), 'Color', plotColor, 'LineWidth', 0.7); grid on; hold on;
            %xlabel('Time (s)');
            ylabel('$\tau_{2}$ (N-m)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Joint 2 Torque','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end

            figure(fignum); nexttile(3); hold on;
            plot(t, 180/pi*y(1,:), 'Color', plotColor, 'LineWidth', 0.7); grid on;
            %xlabel('Time (s)');
            ylabel('$\theta_{1}$ (deg)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Link 1 Angle','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end

            figure(fignum); nexttile(4); hold on;
            plot(t, 180/pi*y(2,:), 'Color', plotColor, 'LineWidth', 0.7); grid on;
            %xlabel('Time (s)');
            ylabel('$\theta_{2}$ (deg)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Link 2 Angle','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end

            figure(fignum); nexttile(5); hold on;
            plot(t, 180/pi*y(3,:), 'Color', plotColor, 'LineWidth', 0.7); grid on;
            xlabel('Time (s)');
            ylabel('$\dot{\theta_{1}}$ (deg/s)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Link 1 Angular Rate','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end

            figure(fignum); nexttile(6); hold on;
            plot(t, 180/pi*y(4,:), 'Color', plotColor, 'LineWidth', 0.7); grid on;
            xlabel('Time (s)'); ylabel('$\dot{\theta_{2}}$ (deg/s)', 'interpreter', 'latex');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
            if makeTitles
                title('Link 2 Angular Rate','FontName','Helvetica Neue','FontSize',14, 'FontWeight', 'normal');
            end
        end
    end
end