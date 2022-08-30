%% CLEAN
clear all
clc
fclose('all');
addpath(genpath(fullfile(pwd,'utils'))); %utilities
%% SETUP
opensim_install = "C:\OpenSim 4.3";
builddir = pwd+"/build/";
cppName ="extendProblem.cpp";
wrapName ="extend_problem.m";
config = "RelWithDebInfo";
%% CMAKE
%if this is failing, check to see if vs 2019 msbuild.exe and cmake are part of
%the system PATH.
system("cmake CmakeLists.txt -S . -B "+builddir);
system("msbuild "+builddir+"customGoals.sln /p:configuration="+config); % 
%% PROCEDURAL CPP CLASS CONSTRUCTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This uses regex to parse the hpp of each custom goal                     %
%and identifies any setter functions along with their arguments, and      %
%automatically generates an extendProblem class.                          % 
%this also procedurally constructs the mex before MEX_DISPATCH            % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
build_extend_class(cppName,wrapName);
%% BUILD MEX
goal_names = get_goal_names('custom_goals');
goal_includes = "-I"""+pwd+goal_names+"""  ";
goal_libs = "-losim"+goal_names+" ";
custom_goal_lib_l = string([goal_libs{:}]);
mex_call = "mex -I""C:\libs"" ";
for gi =goal_includes
    mex_call = mex_call + gi;
end
mex_call = mex_call + "-I"""+opensim_install+"\sdk\spdlog\include"" ";
mex_call = mex_call + "-I"""+opensim_install+"\sdk\Simbody\include"" ";
mex_call = mex_call + "-I"""+opensim_install+"\sdk\include"" ";
mex_call = mex_call + "-I"""+opensim_install+"\sdk\include\OpenSim"" ";
mex_call = mex_call + "-L"""+opensim_install+"\sdk\Simbody\lib"" ";
mex_call = mex_call + "-L"""+ pwd +"\lib\"+config+""" ";
mex_call = mex_call + "-L"""+ pwd +"\bin\"+config+""" ";
mex_call = mex_call + "-lSimTKcommon -lSimTKsimbody -lSimTKmath ";
mex_call = mex_call + "-L"""+opensim_install+"\sdk\lib"" ";
mex_call = mex_call + "-losimActuators -losimExampleComponents -losimSimulation -losimAnalyses -losimJavaJNI -losimTools ";
mex_call = mex_call + custom_goal_lib_l;
mex_call = mex_call + "-losimMoco -losimCommon -losimLepton -losimTools extendProblem.cpp";
eval(mex_call);
