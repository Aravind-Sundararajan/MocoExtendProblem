opensimroot = 'C:\opensim 4.3\'; %create a char array that has the opensim path toplevel directory
addpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add the opensim path to the
javaaddpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add opensimroot bin and the java path to MATLAB's dynamic path path
setenv('PATH', [[opensimroot 'bin'] ';' [opensimroot 'sdk\lib'] ';' getenv('PATH')]);% Set Windows System path to include OpenSim libraries
import org.opensim.modeling.* %import opensim api library'
%% TRACKING
%warning("starting  sim:Tracking");
%WalkSim_Tracking();

%% PREDICTIVE
% sims = enumeration(simulation.EFF);
% if contains(opensimroot, "4.5")
%     warning("Running tests for OpenSim 4.5");   
%     for s = sims'
%         warning("starting sim:" + string(s));
%         WalkSim_predictive(s);
%     end
% else
%     warning("Running tests for pre- OpenSim 4.5");
%     for s = sims'
%         warning("starting sim:" + string(s));
%         WalkSim_predictive_compat(s);  
%     end
% end
%% 
 %WalkSim_predictive(simulation.ZMP);
