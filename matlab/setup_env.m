%% Sets up MATLAB's paths
if ispc
    basePath = 'C:\Users\josiah\ms-thesis';
    cvxPath = fullfile(basePath, 'third-party\cvx-w64');
elseif ismac
    basePath = '/Users/josiah/ms-thesis';
    cvxPath = fullfile(basePath, 'third-party/cvx-maci64');
end
addpath(genpath(fullfile(basePath, 'matlab')));
addpath(cvxPath);