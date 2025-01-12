% build  calls various methods to procedurally constructs the matlab wrapper, the
% mex interface and the cpp class.


%% CLEAN
clear all
clc
fclose('all');

addpath(genpath(fullfile(pwd,'utils'))); %utilities
addpath(genpath(fullfile(pwd,'test'))); %test

%% SETUP
% Option 1: Try to get from environment variable
opensim_install = getenv('OPENSIM_HOME');
if isempty(opensim_install)
    % Option 2: Look for a config file
    if exist('opensim_config.mat', 'file')
        load('opensim_config.mat', 'opensim_install');
    else
        % Option 3: Ask user to select the OpenSim directory
        opensim_install = uigetdir('C:\', 'Select OpenSim Installation Directory');
        if opensim_install == 0
            error('OpenSim installation directory must be specified');
        end
        % Add trailing slash if missing
        if ~endsWith(opensim_install, '\')
            opensim_install = [opensim_install '\'];
        end
        % Optionally save for future use
        save('opensim_config.mat', 'opensim_install');
    end
end

% Validate the path
if ~exist(fullfile(opensim_install, 'bin'), 'dir') || ~exist(fullfile(opensim_install, 'sdk'), 'dir')
    error('Invalid OpenSim installation directory: %s', opensim_install);
end

addpath([opensim_install 'bin'], [opensim_install 'sdk\lib']); % Add OpenSim paths to MATLAB
javaaddpath([opensim_install 'bin'], [opensim_install 'sdk\lib']); % Add Java paths to MATLAB
setenv('PATH', [[opensim_install 'bin'] ';' [opensim_install 'sdk\lib'] ';' getenv('PATH')]); % Set Windows System path to include OpenSim libraries
import org.opensim.modeling.*;

builddir = fullfile(pwd,"build");
bindir = fullfile(pwd,"bin");
solutionPath = fullfile(builddir, "customGoals.sln");
cppName ="extendProblem.cpp";
wrapName ="extend_problem.m";
config ="RelWithDebInfo";
%% CMAKE
%if this is failing, check to see if vs 2022 msbuild.exe and cmake are part of
%the system PATH.
% Extract version number using regex
version_match = regexp(opensim_install, '4\.(\d+)', 'tokens');
if isempty(version_match)
    error('Could not determine OpenSim version from installation path');
end

% Map OpenSim version to OSim_Version value
osim_version = str2double(version_match{1});

% Build cmake command
cmake_cmd = sprintf('cmake CmakeLists.txt -S . -B "%s" -DOSim_Version=%d -G "Visual Studio 17 2022"', ...
    builddir, osim_version);

% Execute cmake
system(cmake_cmd);
system("msbuild """+solutionPath+""" /p:configuration="+config); %
%% PROCEDURAL CPP CLASS CONSTRUCTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This uses regex to parse the hpp of each custom goal                     %
%and identifies any setter functions along with their arguments, and      %
%automatically generates an extendProblem class.                          %
%this also procedurally constructs the mex before MEX_DISPATCH            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
build_extend_class(fullfile(bindir,config,cppName),fullfile(bindir,config,wrapName), opensim_install);
%% BUILD MEX
if osim_version == 5
    goaldir = 'custom_goals';
else
    goaldir = 'custom_goals_compat';
end
goal_names = get_goal_names(goaldir);
goal_includes = "-I"""+pwd+"\" + goaldir+"\"+goal_names+"""  ";
goal_libs = "-losim"+goal_names+" ";
custom_goal_lib_l = string([goal_libs{:}]);
mex_call = "mex -I'"+pwd+"\libs' ";
for gi =goal_includes
    mex_call = mex_call + gi;
end
mex_call = mex_call + "-I'"+opensim_install+"\sdk\spdlog\include' ";
mex_call = mex_call + "-I'"+opensim_install+"\sdk\Simbody\include' ";
mex_call = mex_call + "-I'"+opensim_install+"\sdk\include' ";
mex_call = mex_call + "-I'"+opensim_install+"\sdk\include\OpenSim' ";
mex_call = mex_call + "-L'"+opensim_install+"\sdk\Simbody\lib' ";
mex_call = mex_call + "-L'"+ pwd +"\bin\"+config+"' ";
mex_call = mex_call + "-lSimTKcommon -lSimTKsimbody -lSimTKmath ";
mex_call = mex_call + "-L'"+opensim_install+"\sdk\lib' ";
mex_call = mex_call + "-losimActuators -losimExampleComponents -losimSimulation -losimAnalyses -losimJavaJNI -losimTools ";
mex_call = mex_call + custom_goal_lib_l;
mex_call = mex_call + "-losimMoco -losimCommon -losimLepton -losimTools '"+ pwd+"/bin/"+config+"/"+cppName + "' -outdir '" + pwd+"/bin/"+config+"'";
eval(mex_call);
