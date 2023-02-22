classdef DynamicalSystem
    properties
        % Simulation parameters
        name_str; % string name of system
        opts_ode; % ODE solver options

        % Plant model parameters
        nx {mustBeNumeric}; % number of internal states
        nu {mustBeNumeric}; % number of controlled inputs
        ny {mustBeNumeric}; % number of measured outputs
        nw {mustBeNumeric}; % number of disturbance inputs
        dt {mustBeNumeric}; % sample period (second)
        mu_x {mustBeNumeric}; % mean of measurement noise
        sig2_x {mustBeNumeric}; % variance of measurement noise
        InputNames; % cell array of input name strings
        StateNames; % cell array of input name strings

        umin {mustBeNumeric}; % minimum actuation (vector)
        umax {mustBeNumeric}; % maximum actuation (vector)

        % Linearized dynamic model parameters
        sysc; % continuous-time system
        sysd; % discrete-time system

        % Cost/disturbance weighting matrices
        ctrl; % contains D, H, G, F

        % Frequencies of interest for singular values
        W {mustBeNumeric};

        % ODE solver
        odeSolver;

        % Parameters of training control policy
        Nt {mustBeNumeric};
        K0 {mustBeNumeric};
        mu_exploration;

        % Parameters for simulation
        N {mustBeNumeric};
        x0 {mustBeNumeric};
        u0 {mustBeNumeric};
    end

    methods
        function obj = DynamicalSystem
            obj.InputNames = {};
            obj.StateNames = {};
            obj.opts_ode = odeset(...
                'RelTol', 1e-3, 'AbsTol', 1e-6);
            obj.odeSolver = "ode45";
            obj.Nt = 100;
        end

        % Compute the control input mapping B_tau
        function [B_] = B_tau(obj, q, qdot)
            B_ = zeros(size(q, 1), obj.nu);
        end

        % Compute the mass matrix M(q)
        function [M_] = M(obj, q)
            M_ = zeros(size(q, 1), size(q, 1));
        end

        % Compute the inverse of the mass matrix
        function [invM_] = invM(obj, q)
            invM_ = zeros(size(q, 1), size(q, 1));
        end

        % Compute the Coriolis matrix C(q, qdot)
        function [C_] = C(obj, q, q_dot)
            C_ = zeros(size(q, 1), size(q, 1));
        end

        % Compute linearized version of C(q,qdot)
        function [dC_] = dC(obj, q, q_dot)
            dC_ = zeros(size(q, 1), 1);
        end

        % Compute the gravitational matrix G(q)
        function [g_] = G(obj, q)
            g_ = zeros(size(q, 1), size(q, 1));
        end

        % Compute the partial of G(q) w.r.t. q, for qdot = 0
        function [dg_dq_] = dG_dq(obj, q, q_dot)
            dg_dq_ = zeros(size(q, 1), 1);
        end

        % Continuous-time dynamics of manipulator, in joint coordinates
        function q_ddot = f_manipulator(obj, q, q_dot, tau)
            % Accelerations = M(q)^-1*[g(q) - C(q,qdot)*qdot + tau]
            q_ddot = obj.invM(q)*(obj.G(q) - obj.C(q, q_dot)*q_dot + tau);
        end

        % Continuous-time dynamic model, dx/dt = f(t, x, u)
        function dx = f_ode(obj, t, x, u)
            dx = zeros(obj.nx, 1);
        end

        % Linearize about (q, qdot) to produce dx/dt = A x(t) + B u(t)
        function [obj] = linearize(obj, q, q_dot)
            % Fixed-point linearization (qdot = 0), which is
            % known analytically for the manipulator equation
            if nargin < 2
                q_dot = zeros(size(q, 1), 1);
            end
            if nargin < 1
                q = zeros(obj.nx/2, 1);
            end

            A = [...
                zeros(size(q, 1)), eye(size(q, 1));
                obj.invM(q)*obj.dG_dq(q, q_dot), zeros(size(q,1));
            ];

            B = [...
                zeros(size(q, 1), obj.nu);
                obj.invM(q)*obj.B_tau(q, q_dot);
            ];

            obj.sysc = ss(A, B, eye(obj.nx), 0);
        end

        % Discretize the system with sample period dt
        function [obj] = discretize(obj, dt)
            % Store sample period (sec)
            obj.dt = dt;

            % (Approximate) forward Euler discretization
            %Ad = eye(obj.nx) + obj.dt*obj.sysc.A;
            %Bd = obj.dt*obj.sysc.B;
            %obj.sysd = ss(Ad, Bd, eye(obj.nx), 0, obj.dt);

            % (Exact) input zero-order-hold (ZOH) discretization
            obj.sysd = c2d(obj.sysc, obj.dt);

            % Frequencies of interest for robustness analyses
            obj.W = logspace(log10(1e-3), log10(pi/obj.dt), 200);
        end

        % Compute cost matrices
        function [obj] = set_weights(obj, D)
            if nargin > 1
                % Manually specified
                obj.ctrl.D = D;
            else
                % Unknown disturbances on continuous dynamics, but not kinematics
                D_c = [zeros(obj.nx/2, obj.nx/2); eye(obj.nx/2)]; % continuous
                tmp = ss(obj.sysc.A, [obj.sysc.B, D_c], eye(obj.nx), 0);
                tmpd = c2d(tmp, obj.dt);

                % Linearized, discretized cost/disturbance weighting matrices
                obj.ctrl.D = tmpd.B(:,(obj.nu+1):end);
                if any(obj.ctrl.d)
                    obj.ctrl.D = [obj.ctrl.D, obj.ctrl.d];
                end
            end

            obj.nw = size(obj.ctrl.D,2);
            obj.ctrl.H = [sqrt(obj.ctrl.Qx); zeros(obj.nu, obj.nx)];
            obj.ctrl.G = [zeros(obj.nx, obj.nu); sqrt(obj.ctrl.Ru)];
            obj.ctrl.F = zeros(obj.nx + obj.nu, obj.nw);
        end

        function [P_ol, T_ol, P_cl, T_cl, Lo, So, To] = get_plant(obj, K)
            % Open-loop standard/generic notation: the typical state-space plant,
            % which encodes state dynamics and plant output equations.
            P_ol = ss(obj.sysd.A, obj.sysd.B, eye(obj.nx), 0, obj.dt);

            % Open-loop LFT notation: the "generalized plant" with regulator
            % performance outputs z[k] as well as standard states and measurements.
            A_ol = obj.sysd.A;
            B_ol = [obj.ctrl.D, obj.sysd.B];
            C_ol = [obj.ctrl.H; eye(obj.nx)];
            D_ol = [obj.ctrl.F, obj.ctrl.G; zeros(obj.nx,obj.nu), 0*obj.ctrl.D];
            T_ol = ss(A_ol, B_ol, C_ol, D_ol, obj.dt);

            % Closed-loop plants
            P_cl = [];
            T_cl = [];
            Lo = [];
            So = [];
            To = [];
            if nargin > 1
                % Closed-loop transition matrices
                A_cl = obj.sysd.A + obj.sysd.B*K;
                C_cl = obj.ctrl.H + obj.ctrl.G*K;

                % Closed-loop standard/generic notation: P_ol with state feedback
                P_cl = ss(A_cl, obj.sysd.B, eye(obj.nx), 0, obj.dt);
        
                % Closed-loop generalized/LFT notation: T_ol with state feedback
                T_cl = ss(A_cl, obj.ctrl.D, C_cl, obj.ctrl.F, obj.dt);

                % (Output) loop transfer function matrices for sensitivity analyses
                Lo = ss(obj.sysd.A, obj.sysd.B*K, eye(obj.nx), 0, obj.dt); % = P_ol*K
                So = ss(A_cl, -obj.sysd.B*K, eye(obj.nx), -eye(obj.nx), obj.dt); % = [I + Lo]^-1
                To = ss(A_cl, -obj.sysd.B*K, -eye(obj.nx), 0, obj.dt); % = Lo[I + Lo]^-1
            end
        end

        % Compute one sample of AWG measurement noise
        function [v_k] = v_awgn(obj, t)
            v_k = obj.mu_x + obj.sig2_x.*randn(obj.ny, 1);
        end

        % Compute regulator cost
        function [J, Ju, Jg] = compute_cost(obj, t, u, x, w)
            % Regulator performance
            z = obj.ctrl.H*x + obj.ctrl.G*u;
            N = length(x);
            for k = 1:N
                % Standard and soft-constrained LQ cost
                J(k) = norm(z(:,k), 2)^2;
                Jg(k) = J(k) - obj.ctrl.gamma^2*norm(w(:,k), 2)^2;
        
                % Unweighted control effort alone
                Ju(k) = norm(u(:,k), 2)^2;
            end
            J = cumsum(J);
            Jg = cumsum(Jg);
            Ju = cumsum(Ju);
        end

        % Simulate the dynamical system from x0 for T seconds
        function [t, x, u, y, delta, w] = simulate(obj, x0, u0, N, mu_policy)
            % Pre-allocate data buffers
            t = zeros(1, N+1); % measurement timestamps
            u = zeros(obj.nu, N+1); % controlled inputs
            x = zeros(obj.nx, N+1); % internal states
            y = zeros(obj.ny, N+1); % plant measurements
            delta = zeros(obj.nx, N+1); % state uncertainty
            w = zeros(obj.nw, N+1); % disturbance inputs

            % Main simulation loop
            t(1) = 0;
            x(:,1) = x0;
            u(:,1) = u0;
            for k = 2:N
                % Propagate state dynamics over one measurement period (dt)
                t(k) = t(k-1) + obj.dt;
                if obj.odeSolver == "ode15s"
                    [~, out] = ode15s(@obj.f_ode, [t(k-1) t(k)], x(:,k-1), obj.opts_ode, u(:,k-1));
                elseif obj.odeSolver == "ode23s"
                    [~, out] = ode23s(@obj.f_ode, [t(k-1) t(k)], x(:,k-1), obj.opts_ode, u(:,k-1));
                elseif obj.odeSolver == "ode45"
                    [~, out] = ode45(@obj.f_ode, [t(k-1) t(k)], x(:,k-1), obj.opts_ode, u(:,k-1));
                end
                x(:,k) = out(end,:)';

                % Noisy measurement of full state vector
                y(:,k) = x(:,k) + obj.v_awgn(t(k));

                % Empirical uncertainty over one measurement period (dt)
                delta(:,k) = y(:,k) - (obj.sysd.A*x(:,k-1) + obj.sysd.B*u(:,k-1));
                w(:,k) = obj.ctrl.D\delta(:,k);

                % Saturated output feedback control policy
                u(:,k) = min(obj.umax, max(obj.umin, mu_policy(t(k), y(:,k))));
            end

            % Ignore initial conditions
            t = t(2:end-1);
            x = x(:,2:end-1);
            u = u(:,2:end-1);
            y = y(:,2:end-1);
            delta = delta(:,2:end-1);
            w = w(:,2:end-1);
        end

        % Training control policy
        function [u] = mu_training(obj, t, x)
            u = obj.K0*x + obj.mu_exploration(t, x);
        end

        % Simulate the system to gather training data
        function [t, x, u, y, delta, w] = get_training_data(obj, Nt)
            if nargin < 2
                Nt = obj.Nt;
            end
            [t, x, u, y, delta, w] = obj.simulate(zeros(obj.nx, 1), zeros(obj.nu, 1),...
                Nt, @(t, x) obj.mu_training(t, x));
        end

        function [Stable, DiskMargin, HInfinityNorm, LQCost, LQGammaCost, ControlEffort,...
            DeltaInfNorm, TimeDomainData, FreqDomainData, t, x, u, y, w, SV] = run_test(...
                obj, K, testDir, prefixStr)
            % Only test if the controller is stable
            DiskMargin = NaN;
            HInfinityNorm = NaN;
            LQCost = NaN;
            LQGammaCost = NaN;
            ControlEffort = NaN;
            DeltaInfNorm = NaN;
            TimeDomainData = [];
            FreqDomainData = [];
            t = [];
            x = [];
            u = [];
            y = [];
            w = [];
            SV = [];
            Stable = check_stability(obj.sysd.A, obj.sysd.B, K);
            if Stable
                % Compute linearized plant model(s)
                [P_ol, ~, ~, T_cl] = obj.get_plant(K);

                % Compute disk-based stability margins
                MMIO = diskmargin(P_ol, -1*K);
                DiskMargin = 2*MMIO.DiskMargin; % loop margin w/ simultaneous I/O variation

                % Compute singular values and H-infinity norm of the generalized plant
                SV = sigma(T_cl, obj.W);
                HInfinityNorm = max(SV(1,:)); % = norm(T_cl, Inf)

                % Simulate the plant with closed-loop feedback and some initial conditions
                [t, x, u, y, delta, w] = obj.simulate(obj.x0, obj.u0, obj.N, @(t,x) K*x);
                DeltaInfNorm = norm(delta, Inf);

                % Compare regulator cost and control effort
                [J, Ju, Jg] = obj.compute_cost(t, u, x, w);
                LQCost = J(end);
                LQGammaCost = Jg(end);
                ControlEffort = Ju(end);

                % Aggregate output data
                TimeDomainData = [t; x; u; y; delta; w; J; Ju]; % all transient responses
                FreqDomainData = [obj.W; SV]; % singular values vs. frequency

                if nargin > 5
                    % Store output data to disk
                    writematrix(DiskMargin', fullfile(testDir, [prefixStr '-Disk2MMIO.txt']));
                    writematrix(HInfinityNorm', fullfile(testDir, [prefixStr '-HInfinityNorm.txt']));
                    writematrix(TimeDomainData', fullfile(testDir, [prefixStr '-TimeDomainData.txt']));
                    writematrix(FreqDomainData', fullfile(testDir, [prefixStr '-FreqDomainData.txt']));
                end
            else
                warning('Feedback system is unstable.');
            end
        end

        % Set variance of parameter uncertainty
        function [obj] = parameter_uncertainty(obj, scaleFactor)
            % Set in child classes
        end

        % Set variance of measurement noise
        function [obj] = noise_variance(obj, scaleFactor)
            maxNoiseVariance = obj.ctrl.d;
            obj.sig2_x = scaleFactor*maxNoiseVariance;
        end

        % Add to shared/common plot of integrated metrics (control effort, LQ cost)
        function [] = add_plot_integrated_cost(obj, fignum, t, u, x, w, plotColor)
            [J, Ju] = obj.compute_cost(t, u, x, w);

            figure(fignum); nexttile(1); hold on;
            plot(t, J, 'Color', plotColor, 'LineWidth', 0.7); grid on;
            title('Integrated Linear Quadratic Cost','FontName','Helvetica Neue','FontSize',16);
            ylabel('Cost');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';

            figure(fignum); nexttile(2); hold on;
            plot(t, Ju, 'Color', plotColor, 'LineWidth', 0.7); grid on;
            title('Integrated Control Effort','FontName','Helvetica Neue','FontSize',16);
            ylabel('Effort');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
        end

        % Add to shared/common plot of integrated metrics (control effort, LQ cost)
        function [] = add_plot_average_cost(obj, fignum, t, u, x, w, plotColor)
            [J, Ju] = obj.compute_cost(t, u, x, w);

            k = 1:length(J);
            J = J./k;
            Ju = Ju./k;

            figure(fignum); nexttile(1); hold on;
            plot(t, J, 'Color', plotColor, 'LineWidth', 0.7); grid on;
            title('Average Linear Quadratic Cost','FontName','Helvetica Neue','FontSize',16);
            ylabel('Cost');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';

            figure(fignum); nexttile(2); hold on;
            plot(t, Ju, 'Color', plotColor, 'LineWidth', 0.7); grid on;
            title('Average Control Effort','FontName','Helvetica Neue','FontSize',16);
            ylabel('Effort');
            ax = gca;
            ax.XAxis.FontSize = 14; ax.XAxis.FontName = 'Helvetica Neue';
            ax.YAxis.FontSize = 14; ax.YAxis.FontName = 'Helvetica Neue';
        end

        % Add to shared/common plot of singular values
        function [] = add_plot_singular_values(obj, fignum, tilenum, SV, plotColor, titleStr)
            figure(fignum); nexttile(tilenum); hold on;
            semilogx(obj.W, mag2db(SV(1,:)), 'Color', plotColor);
            title(titleStr, 'FontSize', 14, 'FontWeight', 'normal');
            set(gca,'XScale','log'); xlim([1e-3 pi/obj.dt]); grid on; 
        end
    end
end