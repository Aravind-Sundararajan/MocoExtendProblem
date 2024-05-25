opensimroot = 'C:\opensim 4.5\'; %create a char array that has the opensim path toplevel directory
addpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add the opensim path to the
javaaddpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add opensimroot bin and the java path to MATLAB's dynamic path path
setenv('PATH', [[opensimroot 'bin'] ';' [opensimroot 'sdk\lib'] ';' getenv('PATH')]);% Set Windows System path to include OpenSim libraries
import org.opensim.modeling.* %import opensim api library'
%% TRACKING
WalkSim_Tracking()
%% PREDICTIVE
sims = enumeration(simulation.EFF);
for s = sims'
    WalkSim_predictive(sim_type+1);
end

