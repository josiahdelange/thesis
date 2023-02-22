%% Run everything
% Note: due to the way this thesis was developed, the overall series of plots
% are generated in a somewhat disjoint way.  In order to quickly prototype the
% effect of various tweaks/parameter changes/etc. on various robustness metrics
% (e.g. singular values, or integrated cost), the various plots are generated
% separately.
%
% When run sequentially, there are repeated tests, since many of these functions
% essentially rerun the same overall simulation test harnesses while extracting
% different metrics.  Only if multiple computers are available can the tests be
% run simultaneously.  This script, however, runs them sequentially.
%
clear; clc; %close all;
addpath('utils');
addpath('tests');
addpath('systems');

%% Baseline design plots
% Singular values
plot_waterbed_effect(51, [1, 1.001, 1.01, 1.5]); % Figure 5-1
plot_baseline_singular_values(52, 1.2); % Figure 5-2

% Transient responses
plot_baseline_transient(53, "double-integrator"); % Figure 5-3
plot_baseline_transient(54, "cart-pole"); % Figure 5-4
plot_baseline_transient(55, "robot-arm"); % Figure 5-5

% Integrated performance metrics
plot_baseline_cost(56); % Figure 5-6

%% Results from the simulation test harness
run_all_singular_values([61, 65, 69, 613]); % Figures 6-1, 6-5, 6-9, 6-13
run_all_integrated_metrics([62, 66, 67, 612]); % Figures 6-2, 6-6, 6-7, 6-12
run_all_overall_trends([64, 68, 612, 616]); % Figures 6-4, 6-8, 6-12, 6-16
