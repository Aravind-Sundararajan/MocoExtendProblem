%% Load the Moco libraries
% mex -I"C:\libs"...
% -I"C:\Users\varunjos\AppData\Roaming\MathWorks\MATLAB Add-Ons\Collections\kyamagu_mexplus\include"...
% -I"C:\Users\varunjos\Documents\GitHub\MocoExtendProblem\MocoCoordinateAccelerationGoal"...
% -I"C:\Users\varunjos\Documents\GitHub\MocoExtendProblem\MocoActivationSquaredGoal"...
% -I"C:\Users\varunjos\Documents\GitHub\MocoExtendProblem\MocoZMPGoal"...
% -I"C:\Users\varunjos\Documents\GitHub\MocoExtendProblem\MocoMarkerAccelerationGoal"...
% -I"C:\OpenSim 4.3\sdk\spdlog\include"...
% -I"C:\OpenSim 4.3\sdk\Simbody\include"...
% -I"C:\OpenSim 4.3\sdk\include"...
% -I"C:\OpenSim 4.3\sdk\include\OpenSim"...
% -L"C:\OpenSim 4.3\sdk\Simbody\lib"...
% -L"C:\Users\varunjos\Documents\GitHub\MocoExtendProblem\build\RelWithDebInfo"...
% -L"C:\OpenSim 4.3\sdk\lib"...
% -losimMoco -losimCommon -losimLepton -losimTools...
% -losimActuators -losimExampleComponents -losimSimulation -losimAnalyses -losimJavaJNI...
% -losimMocoActivationSquaredGoal -losimMocoZMPGoal -losimMocoCoordinateAccelerationGoal -losimMocoMarkerAccelerationGoal extendProblem.cpp

%%
opensimroot = 'C:\OpenSim 4.3\'; %create a char array that has the opensim path toplevel directory
addpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add the opensim path to the
javaaddpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add opensimroot bin and the java path to MATLAB's dynamic path path
setenv('PATH', [[opensimroot 'bin'] ';' [opensimroot 'sdk\lib'] ';' getenv('PATH')]);% Set Windows System path to include OpenSim libraries
import org.opensim.modeling.* %import opensim api library

w = 1.0;
mesh_interval = 50;
max_iterations = 15000;
outputDir = './output/';

model = Model('pointmass.osim');

%% Place a marker on the model
bodies = model.getBodySet();
currBody = bodies.get('body1');
testMarker = Marker("testMarker",currBody,Vec3(0,0,0));
model.addMarker(testMarker);
model.finalizeConnections();

% C++ equivalent code for walksim models
% const BodySet& bodySet = model.get_BodySet();
% const PhysicalFrame& torsoBody = bodySet.get(11);
% auto* marker1 = new Marker("headmarker",torsoBody, SimTK::Vec3(0,0.65,0));
% model.addMarker(marker1);
% model.finalizeConnections();

% Create MocoStudy.
% ================
study = MocoStudy();
study.setName('sliding_mass');

%%
% Define the optimal control problem.
% ===================================
problem = study.updProblem();

% Model (dynamics).
% -----------------
problem.setModel(model);

% Bounds.
% -------
% Initial time must be 0, final time can be within [0, 5].
problem.setTimeBounds(MocoInitialBounds(0.0), MocoFinalBounds(1.0));

% Position must be within [-5, 5] throughout the motion.
% Initial position must be 0, final position must be 1.
problem.setStateInfo('/slider/position/value', MocoBounds(0, 1), MocoInitialBounds(0), MocoFinalBounds(1));

% Speed must be within [-50, 50] throughout the motion.
% Initial and final speed must be 0. Use compact syntax.
problem.setStateInfo('/slider/position/speed', [-50, 50],[] , []);

% Applied force must be between -50 and 50.
problem.setControlInfo('/actuator', MocoBounds(-50, 50));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Adding the marker goal - Matt look here %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cptr = uint64(problem.getCPtr(problem));
ep = extend_problem(cptr);

ep.addMarkerGoal(1.0,'/markerset/testMarker',true);
ep.addAccelerationGoal(1.0,{'/slider/position'},true);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(50);

% Solve the problem.
% ==================
solution = study.solve();
solution.write('sliding_mass_solution.sto');
dur = seconds(solution.getSolverDuration());
[h,m,s] = hms(dur);
disp('   ')
disp(['Solver duration (h:m:s): ' num2str(h) ':' num2str(m) ':' num2str(round(s))])
disp('   ')
disp('Breakdown of objective (including weights):')

try
    for i = 1:solution.getNumObjectiveTerms()
        termName = solution.getObjectiveTermNames().get(i-1);
        termNamestr = termName.toCharArray';
        disp(['  ',termNamestr,': ',num2str(solution.getObjectiveTermByIndex(i-1)) ])
    end
catch
    for i = 1:solution.getNumObjectiveTerms()
        termName = solution.getObjectiveTermNames().get(i-1);
        disp(['  ',termName,': ',num2str(solution.getObjectiveTermByIndex(i-1)) ])
    end
end

ep.delete();
