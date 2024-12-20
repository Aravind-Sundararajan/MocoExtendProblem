% OpenSim Simulation Pipeline - Tracking and Predictive Simulations
%
% This script configures the MATLAB environment to run OpenSim simulations 
% for gait tracking and predictive modeling. It supports both OpenSim 4.5 
% and earlier versions by dynamically adjusting compatibility functions.
%
% Key Features:
% 1. OpenSim Path Setup:
%    - Adds the OpenSim bin and SDK library paths to MATLAB's search path.
%    - Configures the Windows system PATH to include OpenSim libraries.
%    - Imports the OpenSim API for MATLAB usage.
% 2. Tracking Simulation:
%    - Runs the `WalkSim_Tracking` function to perform optimal control 
%      simulations for tracking gait dynamics.
% 3. Predictive Simulations:
%    - Runs predictive simulations using the `WalkSim_predictive` function.
%    - Supports different predictive simulation modes enumerated in the 
%      `simulation.EFF` enumeration class.
%    - Ensures compatibility with OpenSim versions pre-4.5 using 
%      `WalkSim_predictive_compat`.
%
% Workflow:
% - OpenSim 4.5-specific tests are run if the `opensimroot` contains "4.5".
% - A warning is issued at the start of each simulation for easier tracking 
%   of progress in the MATLAB command window.
%
% Inputs:
% - `opensimroot` (char array): Path to the top-level OpenSim directory.
%
% Dependencies:
% - OpenSim 4.x installed on the system.
% - WalkSim_Tracking.m and WalkSim_predictive.m available in the MATLAB path.
%
% Notes:
% - Ensure the correct `opensimroot` path is set for your OpenSim installation.
% - Modify the `simulation.EFF` enumeration class to include the predictive 
%   modes relevant to your study.
%
% Example:
% - To execute this script, simply run it in MATLAB after ensuring the 
%   OpenSim 4.x installation and MATLAB bindings are properly configured.
%
% Author: A. Sundararajan, MWU
% Contributors: Alex Denton & Brian Umberger
% Date: 12/20/2024

opensimroot = 'C:\opensim 4.5\'; %create a char array that has the opensim path toplevel directory
addpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add the opensim path to the
javaaddpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add opensimroot bin and the java path to MATLAB's dynamic path path
setenv('PATH', [[opensimroot 'bin'] ';' [opensimroot 'sdk\lib'] ';' getenv('PATH')]);% Set Windows System path to include OpenSim libraries
import org.opensim.modeling.* %import opensim api library'
%% TRACKING
warning("starting  sim:Tracking");
WalkSim_Tracking();

%% PREDICTIVE
sims = enumeration(simulation.EFF);
if contains(opensimroot, "4.5")
    warning("Running tests for OpenSim 4.5");
    for s = sims'
        warning("starting sim:" + string(s));
        WalkSim_predictive(s);
    end
else
    warning("Running tests for pre- OpenSim 4.5");
    for s = sims'
        warning("starting sim:" + string(s));
        WalkSim_predictive_compat(s);
    end
end

% WalkSim_predictive(simulation.ZMP);
