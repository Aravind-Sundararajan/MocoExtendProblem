function WalkSim_predictive(sim_type)

%------------------------------------------------------------------------
% WalkSim_predictive: Predictive Gait Simulation
% 
% This function solves a predictive walking optimization problem using
% OpenSim's Moco framework. The simulation minimizes discrepancies between
% simulated and reference ground reaction forces (GRFs), coordinate values,
% and speeds, while minimizing control effort.
%
% The optimization uses Nguyen et al. (2019) as reference data, simulating
% half a gait cycle with symmetry enforced between left and right limbs.
%
% Key Features:
% 1. **Effort Minimization**: Minimizes muscle control effort.
% 2. **Symmetry Goal**: Enforces symmetry in joint kinematics and muscle
%    activations to simulate half a step.
% 3. **Adaptive Mesh**: Adjustable temporal mesh intervals, guess strategies, 
%    and core usage for enhanced optimization efficiency.
% 4. **Multiple Predictive Goals**: Offers options to predict gait outcomes 
%    like marker accelerations, base of support (BOS), or zero-moment-point 
%    (ZMP), based on user input.
%
% Simulation Parameters:
% - sim_type: Defines the predictive goal type (Effort, Marker Acceleration, 
%             BOS, or ZMP). This is an enumeration from the simulation.m in 
%             the toplevel directory that is spectified like simulation.EFF.
% - mesh_int: Number of mesh intervals for time discretization.
%
% The code utilizes custom-defined goals through the extended problem
% function and applies bounds on joint states and muscle activations.
%
% Author: Aravind Sundararajan
% Date: 12/20/2024
%------------------------------------------------------------------------

addpath(genpath(fullfile(pwd,'utils'))); %utilities
addpath(genpath(fullfile(pwd,'test'))); %test
addpath(genpath(fullfile(pwd,'models'))); %utilities
addpath(genpath(fullfile(pwd,'sandbox'))); %sandbox
addpath(genpath(fullfile(pwd,'bin','RelWithDebInfo'))); %Extend Problem (magic!)
mesh_int= 25;

output_dirs = ["./output/effpred/",...
    "./output/meppredmarkerAccel/",...
    "./output/meppredBOS/",...
    "./output/meppredZMP/",...
    ];
AccelerationWeights = [ (1e-3)/11 (1e-3)/11 (1e-3)/11 (1e-3)/11]; %(1e-11)/4 (1e-11)/4 (1e-5)/2 (1e-11)/4 (1e-6)/4 ];
goal_type = sim_type + 1;
output_dir =output_dirs(goal_type);
acceleration_weight=AccelerationWeights(goal_type);


% Load the Moco libraries
import org.opensim.modeling.*;

% Define the optimal control problem
% ==================================
study = MocoStudy();
study.setName('gaitPredictive');
model = Model('.\test\example2DWalking\WalkSim_11DOF_18Mus_05_02_2018.osim');

% Set the weights for the terms in the objective function.
%
% The control effort and tracking weights were set through trial
% and error to give a reasonable balance between minimizing effort
% and matching the reference data well. The weight on the term that
% minimizes the derivatives of the auxiliary variables has been set
% as low as possible while still resulting in smooth muscle activations
% and tendon forces.
ControlEffortWeight =  10.0;
StateTrackingWeight =  0.0;
GRFTrackingWeight   =  0.0;
AuxDerivWeight      =  0.001;


% Reference data for tracking problem
tableProcessor = TableProcessor('.\output\track\WalkSimReferenceCoordinates.sto');
tableProcessor.append(TabOpLowPassFilter(6));

% More MocoTrack settings
modelProcessor = ModelProcessor(model);

problem = study.updProblem();
problem.setModelProcessor(modelProcessor);
problem.setTimeBounds(MocoInitialBounds(0.0), MocoFinalBounds(0.54064765));

% Goals
% =====

% Symmetry (to permit simulating only one step)
symmetryGoal = MocoPeriodicityGoal('symmetryGoal');
problem.addGoal(symmetryGoal);
model = modelProcessor.process();
model.initSystem();

% Symmetric coordinate values (except for pelvis_tx) and speeds
for i = 1:model.getNumStateVariables()
    currentStateName = string(model.getStateVariableNames().getitem(i-1));
    if startsWith(currentStateName , '/jointset')
        if contains(currentStateName,'_r')
            symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                regexprep(currentStateName,'_r','_l') ) );
        end
        if contains(currentStateName,'_l')
            symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                regexprep(currentStateName,'_l','_r') ) );
        end
        if (~contains(currentStateName,'_r') && ~contains(currentStateName,'_l') && ...
                ~contains(currentStateName,'pelvis_tx/value')  && ...
                ~contains(currentStateName,'/activation'))
            symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
        end
    end
end

% Symmetric muscle activations
for i = 1:model.getNumStateVariables()
    currentStateName = string(model.getStateVariableNames().getitem(i-1));
    if endsWith(currentStateName,'/activation')
        if contains(currentStateName,'_r')
            symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                regexprep(currentStateName,'_r','_l')));
        end
        if contains(currentStateName,'_l')
            symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                regexprep(currentStateName,'_l','_r')));
        end

    end
end

% Symmetric tendon forces
for i = 1:model.getNumStateVariables()
    currentStateName = string(model.getStateVariableNames().getitem(i-1));
    if endsWith(currentStateName,'/normalized_tendon_force')
        if contains(currentStateName,'_r')
            symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                regexprep(currentStateName,'_r','_l')));
        end
        if contains(currentStateName,'_l')
            symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                regexprep(currentStateName,'_l','_r')));
        end

    end
end

% Symmetric controls
controlsNames = problem.createRep().createControlInfoNames();
for i = 1:model.getNumControls()
    currentControlName = string(controlsNames.get(i-1));
    if (contains(currentControlName,'_r') || contains(currentControlName,'_l'))

        if contains(currentControlName,'_r')
            symmetryGoal.addControlPair(MocoPeriodicityGoalPair(currentControlName , ...
                regexprep(currentControlName,'_r','_l')));
        end
        if contains(currentControlName,'_l')
            symmetryGoal.addControlPair(MocoPeriodicityGoalPair(currentControlName , ...
                regexprep(currentControlName,'_l','_r')));
        end

    end
end

% Prescribed average gait speed
speedGoal = MocoAverageSpeedGoal('speed');
problem.addGoal(speedGoal);
speedGoal.set_desired_average_speed(1.3);

% Get a reference to the MocoControlGoal that is added to every MocoTrack
% problem by default and change the weight
effortGoal = MocoControlGoal('effort', 10);
problem.addGoal(effortGoal);
effortGoal.setExponent(2);

left_foot = char(model.getBodySet().get("calcn_l").getAbsolutePathString());
right_foot = char(model.getBodySet().get("calcn_r").getAbsolutePathString());

cptr = uint64(problem.getCPtr(problem));
ep = extend_problem(cptr);


div_disp =false;
div_dur =false;
div_mass =false;

if goal_type == 1
    %eff pred
    disp("ACT")
    custom_goal_name = 'act';
    musc_names = {char(model.getMuscles().get(0).getAbsolutePath().toString()),...
                  char(model.getMuscles().get(9).getAbsolutePath().toString())};
    musc_weights = [2000.0,2000.0];
    ep.addMocoActivationGoal(custom_goal_name, 1.0, false, false, false, false, 2,musc_names,musc_weights);
elseif goal_type ==2
%     ep.addMocoMarkerAccelerationGoal('addMocoMarkerAccelerationGoal',10.0,...
%         div_disp, div_dur, div_mass,...
%         char(model.getMarkerSet().get("head_marker").getAbsolutePathString()));
    ep.addMocoCoordinateAccelerationGoal('addMocoCoordinateAccelerationGoal',10.0,...
        div_disp, div_dur, div_mass,2,...
        {char(model.getCoordinateSet().get(0).getAbsolutePath().toString),...
        char(model.getCoordinateSet().get(0).getAbsolutePath().toString)}, ...
        [1.0,1.0]);
elseif goal_type ==3
    ep.addMocoBOSGoal('base_of_support',10.0,...
        div_disp, div_dur,div_mass, 1,...
        left_foot, right_foot);
elseif goal_type ==4
    ep.addMocoZMPGoal('zero_moment_point',10.0,...
        div_disp, div_dur, div_mass, 1);
end

% Bounds
% ======
% t_final is fixed in the tracking problem
problem.setStateInfo("/jointset/ground_pelvis/pelvis_tilt/value", [-20*pi/180, 10*pi/180]);
problem.setStateInfo("/jointset/ground_pelvis/pelvis_tx/value", [-1, 2], 0, [0.5, 1.0]);
problem.setStateInfo("/jointset/ground_pelvis/pelvis_ty/value", [0.75, 1.25]);
problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value", [-20*pi/180, 60*pi/180]);
problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value", [-20*pi/180, 60*pi/180]);
problem.setStateInfo("/jointset/knee_l/knee_angle_l/value", [-80*pi/180, 10*pi/180]);
problem.setStateInfo("/jointset/knee_r/knee_angle_r/value", [-80*pi/180, 10*pi/180]);
problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value", [-35*pi/180, 25*pi/180]);
problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value", [-35*pi/180, 25*pi/180]);
problem.setStateInfo("/jointset/mtp_r/mtp_angle_r/value", [-25*pi/180, 75*pi/180]);
problem.setStateInfo("/jointset/mtp_l/mtp_angle_l/value", [-25*pi/180, 75*pi/180]);

% Muscle bounds
problem.setStateInfoPattern('/forceset/.*/normalized_tendon_force', [0, 1.5], [], []);
problem.setStateInfoPattern('/forceset/.*/activation',  [0.01, 1.0], [], []);
problem.setControlInfoPattern('.*', [0.01, 1.0], [], []);



% Configure the solver
% ====================
solver = MocoCasADiSolver.safeDownCast(study.updSolver());
solver.resetProblem(problem);
solver.set_verbosity(2); % increase this value for more dense output
solver.set_optim_solver("ipopt");
solver.set_optim_constraint_tolerance(1e-3);
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_max_iterations(10000);
solver.set_minimize_implicit_auxiliary_derivatives(true);
solver.set_implicit_auxiliary_derivatives_weight(AuxDerivWeight);

solver.set_multibody_dynamics_mode('implicit')
solver.set_minimize_implicit_multibody_accelerations(true)
solver.set_implicit_multibody_accelerations_weight(acceleration_weight)
solver.set_implicit_multibody_acceleration_bounds(MocoBounds(-250, 250))

guess_track = MocoTrajectory('.\output\track\states_half.sto');
guess = solver.createGuess();

trackStatesTable = guess_track.exportToStatesTable();
trackControlsTable = guess_track.exportToControlsTable();


guess.insertStatesTrajectory(trackStatesTable, true);
guess.insertControlsTrajectory(trackControlsTable, true);
solver.setGuess(guess)


% The next 2 items are specific to the conditions that were evaluated in
% the corresponding paper (Denton & Umberger, 2023)

% 1) Set the number of mesh intervals for this case
solver.set_num_mesh_intervals(mesh_int);

% 2) Set the number of cores to use
%
% The following code handles the fact that serial operation is actually
% achieved using "set_parallel(0)" while "set_parallel(1)" would use all
% available cores
solver.set_parallel(1);

% Solve the problem
% =================
gaitPredictiveSolution = study.solve();

% Extract some information from the solution
output(1) = gaitPredictiveSolution.getObjective();
output(2) = gaitPredictiveSolution.getSolverDuration();
output(3) = gaitPredictiveSolution.getNumIterations();

% Save the solution with # of mesh intervals and processor cores in the filename

% Also save this solution with a generic filename that can be used as the
% initial guess on the next finest mesh density
gaitPredictiveSolution.write(output_dir + 'states_half.sto');



% Optional output: Everything from here to the end is only for animation
% and plotting purposes and could be commented out or deleted.

% Create a full stride from the periodic single step solution
fullStride = opensimMoco.createPeriodicTrajectory(gaitPredictiveSolution);
fullStride.write(output_dir + 'states2.sto');

% Extract ground reaction forces
% ==============================
contactSpheres_r = StdVectorString();
contactSpheres_l = StdVectorString();
contactSpheres_r.add("Contact_Foot_Ground_R1");
contactSpheres_r.add("Contact_Foot_Ground_R2");
contactSpheres_r.add("Contact_Foot_Ground_R3");
contactSpheres_r.add("Contact_Foot_Ground_R4");
contactSpheres_r.add("Contact_Foot_Ground_R5");
contactSpheres_r.add("Contact_Foot_Ground_R6");
contactSpheres_r.add("Contact_Foot_Ground_R7");
contactSpheres_r.add("Contact_Foot_Ground_R8");
contactSpheres_l.add("Contact_Foot_Ground_L1");
contactSpheres_l.add("Contact_Foot_Ground_L2");
contactSpheres_l.add("Contact_Foot_Ground_L3");
contactSpheres_l.add("Contact_Foot_Ground_L4");
contactSpheres_l.add("Contact_Foot_Ground_L5");
contactSpheres_l.add("Contact_Foot_Ground_L6");
contactSpheres_l.add("Contact_Foot_Ground_L7");
contactSpheres_l.add("Contact_Foot_Ground_L8");

externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(model, ...
    fullStride,contactSpheres_r,contactSpheres_l);
STOFileAdapter.write(externalForcesTableFlat, ...
    output_dir + 'GRF.sto');

ref = MocoTrajectory(output_dir + '/outputReference/states_half.sto');

if gaitPredictiveSolution.isNumericallyEqual(ref, 0.1)
    warning("output matches output reference for " + output_dir);
else
    warning("reference failed to match reference output for goal " + output_dir);
end

end
