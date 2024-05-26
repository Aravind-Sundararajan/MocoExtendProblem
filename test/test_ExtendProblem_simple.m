opensimroot = 'C:\opensim 4.5\'; %create a char array that has the opensim path toplevel directory
addpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add the opensim path to the
javaaddpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add opensimroot bin and the java path to MATLAB's dynamic path path
setenv('PATH', [[opensimroot 'bin'] ';' [opensimroot 'sdk\lib'] ';' getenv('PATH')]);% Set Windows System path to include OpenSim libraries
import org.opensim.modeling.* %import opensim api library'

addpath(genpath(fullfile(pwd,'utils'))); %utilities
addpath(genpath(fullfile(pwd,'models'))); %utilities
addpath(genpath(fullfile(pwd,'sandbox'))); %sandbox
addpath(genpath(fullfile(pwd,'bin','RelWithDebInfo'))); %Extend Problem (magic!)

%% loading dlls
% goals = get_goal_names('custom_goals');
% goals = pwd+"bin/RelWithDebInfo/osim" + goals + ".dll";
% for g = goals
%     opensimCommon.LoadOpenSimLibraryExact(g);
% end
%adding extendProblem class path
%setup
w = 1.0;
mesh_interval = 50;
max_iterations = 15000;
outputDir = './test/';
p = createPointMass('./models/pointmass.osim', opensimroot);

model = Model(p);

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
problem.setStateInfo('/slider/position/value', MocoBounds(0, 10), MocoInitialBounds(0), MocoFinalBounds(2));

% Speed must be within [-50, 50] throughout the motion.
% Initial and final speed must be 0. Use compact syntax.
problem.setStateInfo('/slider/position/speed', [-250, 250],0 , 0);

% Applied force must be between -50 and 50.
problem.setControlInfo('/actuator', MocoBounds(-250, 250));

cptr = uint64(problem.getCPtr(problem));
ep = extend_problem(cptr);

for j = 1:2
if j == 1
    %no custom goal
else
    %ep.addMocoCoordinateAccelerationGoal('coordinate_acceleration_goal',1.0,true,{'/slider/position'});
    %ep.addMocoActivationSquaredGoal('act_square',1.0, true, 0.);
    %ep.addMocoMarkerAccelerationGoal('marker_acceleration_goal',1.0,'/markerset/testMarker',true);
    ep.addMocoMaxCoordinateGoal('max_coordinate_goal',1.0, false, 'position');
end


solver = study.initCasADiSolver();
guess = solver.createGuess();
numRows = guess.getNumTimes();
guess.setState('/slider/position/value', linspace(0,0,numRows));
guess.setState('/slider/position/speed', linspace(0,0,numRows));
solver.setGuess(guess);
%solver.set_num_mesh_intervals(50);
%solver.set_optim_constraint_tolerance(1e-10);
%solver.set_optim_convergence_tolerance(1e-10);
% Solve the problem.
% ==================
solution = study.solve();

if j == 1
    solution.write('./test/pointMass/sliding_mass_solution_NoMax.sto');
    ref = MocoTrajectory('./test/pointMass/outputReference/sliding_mass_solution_NoMax.sto');
    
    if solution.isNumericallyEqual(ref)
        disp("1) output matches output reference for NoMax");
    else
        disp("failed to match reference output for goal");
    end

else
    solution.write('./test/pointMass/sliding_mass_solution.sto');
    ref = MocoTrajectory('./test/pointMass/outputReference/sliding_mass_solution.sto');

    if solution.isNumericallyEqual(ref)
        disp("2) output matches output reference for MaxGoal");
    else
        disp("max goal failed to match reference output for goal");
    end
end

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


end

d = ReadOpenSimData('./test/pointMass/sliding_mass_solution.sto');
d2 = ReadOpenSimData('./test/pointMass/sliding_mass_solution_NoMax.sto');
plot(d.data(:,1),d.data(:,2), 'LineWidth',2); hold on;
plot(d2.data(:,1),d2.data(:,2), 'LineWidth',2);

title("Point Mass with MEP's Maximize Coordinate Goal",'FontName','Times New Roman');
xlabel('Time (s)','FontName','Times New Roman');
ylabel('Position (m)','FontName','Times New Roman');
xlim([-0.25 1.25]);
ylim([-1 12]);
legend('MaxCoordinateGoal', 'position and speed constraints only', 'FontName', 'Times New Roman');


ep.delete();
