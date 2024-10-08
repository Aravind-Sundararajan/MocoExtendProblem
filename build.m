% build  calls various methods to procedurally constructs the matlab wrapper, the
% mex interface and the cpp class.


%% CLEAN
clear all
clc
fclose('all');
addpath(genpath(fullfile(pwd,'utils'))); %utilities
%% SETUP
opensim_install = 'C:\opensim 4.5\'; % Path to the top-level OpenSim directory
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
if contains(opensim_install,"4.5")
    system("cmake CmakeLists.txt -S . -B """+builddir+""" -DOSim_Version=5 -G ""Visual Studio 17 2022""");
elseif contains(opensim_install,"4.4")
    system("cmake CmakeLists.txt -S . -B """+builddir+""" -DOSim_Version=4 -G ""Visual Studio 17 2022""");
elseif contains(opensim_install,"4.3")
    system("cmake CmakeLists.txt -S . -B """+builddir+""" -DOSim_Version=3 -G ""Visual Studio 17 2022""");
elseif contains(opensim_install,"4.2")
    system("cmake CmakeLists.txt -S . -B """+builddir+""" -DOSim_Version=2 -G ""Visual Studio 17 2022""");
    %system("cmake CmakeLists.txt -S . -B """+builddir+"""");
end
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
if contains(opensim_install, "4.5")
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
