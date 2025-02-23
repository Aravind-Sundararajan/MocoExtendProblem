% =========================================================================
% Test Script: Point Mass Optimization using OpenSim Moco
% =========================================================================
% Description:
% This script performs an optimal control simulation for a point mass model
% using OpenSim's Moco framework. The test evaluates the influence of 
% including or excluding a custom goal (MaxCoordinateGoal) on the 
% optimization results. The main objectives include:
% 1. Defining and solving an optimal control problem for a sliding mass.
% 2. Comparing simulation results with and without the MaxCoordinateGoal.
% 3. Validating numerical outputs against reference solutions.
% 
% Key Features:
% - Dynamically configures the OpenSim environment within MATLAB.
% - Sets up a point mass model with specified state and control bounds.
% - Utilizes ExtendProblem to include custom optimization goals.
% - Outputs solver duration and a breakdown of objective terms.
% - Plots and compares simulation results for visualization.
%
% Required Setup:
% - OpenSim 4.5 or compatible version installed and accessible.
% - Path to OpenSim libraries correctly set in 'opensimroot'.
% - Point mass model file ('pointmass.osim') located in './models/'.
%
% Outputs:
% - Simulation result files written to './test/pointMass/'.
% - Comparison plots of position vs. time for different test cases.
%
% Author: Aravind Sundararajan
% Date: 12/20/2024
% =========================================================================



% Load the Moco libraries
% Try to get OpenSim path from environment variable first
opensimroot = getenv('OPENSIM_HOME');
if isempty(opensimroot)
    % Try to load from config file if it exists
    if exist('opensim_config.mat', 'file')
        load('opensim_config.mat', 'opensim_install');
        opensimroot = opensim_install;
    else
        error('OpenSim installation path not found. Set OPENSIM_HOME environment variable or create opensim_config.mat');
    end
end

% Add trailing slash if missing
if ~endsWith(opensimroot, '\')
    opensimroot = [opensimroot '\'];
end

addpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); 
javaaddpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); 
setenv('PATH', [[opensimroot 'bin'] ';' [opensimroot 'sdk\lib'] ';' getenv('PATH')]);
import org.opensim.modeling.*


addpath(genpath(fullfile(pwd,'utils'))); %utilities
addpath(genpath(fullfile(pwd,'test'))); %test
addpath(genpath(fullfile(pwd,'models'))); %utilities
addpath(genpath(fullfile(pwd,'sandbox'))); %sandbox
addpath(genpath(fullfile(pwd,'bin','RelWithDebInfo'))); %Extend Problem (magic!)

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
        %custom_goal_name = 'coordinate_acceleration_goal';
        %ep.addMocoCoordinateAccelerationGoal(custom_goal_name,1.0,true, false, false,{'/slider/position'});
        
        %custom_goal_name = 'act_square';
        %ep.addMocoActivationSquaredGoal(custom_goal_name, 1.0, false, false, false, false);
        
        %custom_goal_name = 'marker_acceleration_goal';
        %ep.addMocoMarkerAccelerationGoal(custom_goal_name,w,false,true,false,'/markerset/testMarker');
        
        custom_goal_name = 'max_coordinate_goal';
        ep.addMocoMaxCoordinateGoal(custom_goal_name,w, false, false, false, 'position');
    end



    solver = study.initCasADiSolver();
    guess = solver.createGuess();
    numRows = guess.getNumTimes();
    guess.setState('/slider/position/value', linspace(0,0,numRows));
    guess.setState('/slider/position/speed', linspace(0,0,numRows));
    solver.setGuess(guess);
    % Solve the problem.
    % ==================
    solution = study.solve();

    if j == 1
        solution.write('./test/pointMass/sliding_mass_solution_NoMax.sto');
        ref = MocoTrajectory('./test/pointMass/outputReference/sliding_mass_solution_NoMax.sto');

        if solution.isNumericallyEqual(ref, 0.1)
            disp("1) output matches output reference for NoMax");
        else
            disp("failed to match reference output for goal");
        end

    else
        solution.write('./test/pointMass/sliding_mass_solution.sto');
        ref = MocoTrajectory('./test/pointMass/outputReference/sliding_mass_solution.sto');

        if solution.isNumericallyEqual(ref, 0.1)
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

title("Point Mass with MEP's " + string(custom_goal_name),'FontName','Times New Roman', 'Interpreter','none');
xlabel('Time (s)','FontName','Times New Roman');
ylabel('Position (m)','FontName','Times New Roman');
xlim([-0.25 1.25]);
ylim([-1 12]);
legend(string(custom_goal_name), 'position and speed constraints only', 'FontName', 'Times New Roman');


ep.delete();
