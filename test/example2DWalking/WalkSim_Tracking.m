function WalkSim_Tracking()

% WalkSim_Tracking - Solve a gait tracking optimal control problem
%
% This function sets up and solves an optimal control problem for tracking
% human gait dynamics using OpenSim's Moco framework. The problem seeks to 
% minimize the deviation between simulated and reference data for joint 
% coordinates, speeds, and ground reaction forces (GRFs), while minimizing 
% effort (squared controls). The reference data is taken from Nguyen et al. 
% (2019, IEEE TNSRE) and represents half a gait cycle.
%
% Key Features:
% 1. Symmetry Constraints: Enforces symmetry across state variables, 
%    controls, and tendon forces, permitting simulation of only one step.
% 2. GRF Tracking: Includes a contact tracking goal for ground reaction forces.
% 3. Gait Speed Enforcement: Prescribes a desired average walking speed.
% 4. Effort Optimization: Balances tracking accuracy and control effort using 
%    user-defined weights.
% 5. Mesh Refinement and Guess Strategies: Supports adjustable temporal 
%    resolution and initial guess strategies.
%
% Inputs:
% None (Parameters are hardcoded within the script for setup).
%
% Outputs:
% 1. Saves the optimal control solution to a specified directory.
% 2. Outputs key metrics including the objective value, solver duration, 
%    and the number of iterations.
%
% References:
% - Nguyen et al., 2019, IEEE TNSRE: Reference data source for joint states
%   and GRFs.
% - Denton & Umberger, 2023: Detailed methodology and parameter tuning.
%
% Notes:
% - Requires OpenSim Moco libraries and relevant input files for the model 
%   and reference data.
% - The function is designed for 2D gait dynamics with a predefined model 
%   and boundary conditions.
%
% Authors: Alex Denton & Brian Umberger
% Contributor: A. Sundarajan, MWU (Custom Goals)

mesh_int= 25;
cores = 1;
guess_strategy ='CG';
coarsest_mesh = 25;
output_dirs = ["./output/track/"];

output_dir =output_dirs(1);



% Load the Moco libraries
import org.opensim.modeling.*;

% Define the optimal control problem
% ==================================
track = MocoTrack();
track.setName('gaitTracking');
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
StateTrackingWeight =  1.0;
GRFTrackingWeight   =  0.1;
AuxDerivWeight      =  0.001;


% Reference data for tracking problem
tableProcessor = TableProcessor('.\test\example2DWalking\WalkSimReferenceCoordinates.sto');
tableProcessor.append(TabOpLowPassFilter(6));

% More MocoTrack settings
modelProcessor = ModelProcessor(model);
track.setModel(modelProcessor);
track.setStatesReference(tableProcessor);
track.set_states_global_tracking_weight(StateTrackingWeight);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);
track.set_initial_time(0.0);
track.set_final_time(0.54064765);

% Set the initial guess based on whether we are using the consistent
% guess (CG) approach or the mesh refinement (MR) approach.
% See the publication for more details.


% Always use the provided guess file for the intial guess
track.set_guess_file('.\test\example2DWalking\InitialGuessFileForTracking.sto');

% Adjust the weights so some specific coordinates are not tracked
coordWeights = MocoWeightSet();
coordWeights.cloneAndAppend(MocoWeight('/jointset/mtp_r/mtp_angle_r/value', 0));
coordWeights.cloneAndAppend(MocoWeight('/jointset/mtp_l/mtp_angle_l/value', 0));
coordWeights.cloneAndAppend(MocoWeight('/jointset/mtp_r/mtp_angle_r/speed', 0));
coordWeights.cloneAndAppend(MocoWeight('/jointset/mtp_l/mtp_angle_l/speed', 0));
coordWeights.cloneAndAppend(MocoWeight('/jointset/ground_pelvis/pelvis_tx/value', 0));
coordWeights.cloneAndAppend(MocoWeight('/jointset/ground_pelvis/pelvis_tx/speed', 0));
track.set_states_weight_set(coordWeights);

study = track.initialize();
problem = study.updProblem();


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
effort = MocoControlGoal.safeDownCast(problem.updGoal("control_effort"));
effort.setWeight(ControlEffortWeight);

% Contact (GRF) tracking goal
contactTracking = MocoContactTrackingGoal('contact',GRFTrackingWeight);
contactTracking.setExternalLoadsFile('.\test\example2DWalking\WalkSimReferenceGRF.xml');

forceNamesRightFoot = StdVectorString();
forceNamesRightFoot.add("Contact_Foot_Ground_R1");
forceNamesRightFoot.add("Contact_Foot_Ground_R2");
forceNamesRightFoot.add("Contact_Foot_Ground_R3");
forceNamesRightFoot.add("Contact_Foot_Ground_R4");
forceNamesRightFoot.add("Contact_Foot_Ground_R5");
forceNamesRightFoot.add("Contact_Foot_Ground_R6");
forceNamesRightFoot.add("Contact_Foot_Ground_R7");
forceNamesRightFoot.add("Contact_Foot_Ground_R8");
altFramesRightFoot = StdVectorString();
altFramesRightFoot.add('/bodyset/toes_r');
contactTracking.addContactGroup(MocoContactTrackingGoalGroup(forceNamesRightFoot,'Right_GRF',altFramesRightFoot));

forceNamesLeftFoot = StdVectorString();
forceNamesLeftFoot.add("Contact_Foot_Ground_L1");
forceNamesLeftFoot.add("Contact_Foot_Ground_L2");
forceNamesLeftFoot.add("Contact_Foot_Ground_L3");
forceNamesLeftFoot.add("Contact_Foot_Ground_L4");
forceNamesLeftFoot.add("Contact_Foot_Ground_L5");
forceNamesLeftFoot.add("Contact_Foot_Ground_L6");
forceNamesLeftFoot.add("Contact_Foot_Ground_L7");
forceNamesLeftFoot.add("Contact_Foot_Ground_L8");
altFramesLeftFoot = StdVectorString();
altFramesLeftFoot.add('/bodyset/toes_l');
contactTracking.addContactGroup(MocoContactTrackingGoalGroup(forceNamesLeftFoot,'Left_GRF',altFramesLeftFoot));

contactTracking.setProjection('plane');
contactTracking.setProjectionVector(Vec3(0, 0, 1));
problem.addGoal(contactTracking);


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
solver.set_optim_max_iterations(5000);
solver.set_minimize_implicit_auxiliary_derivatives(true);
solver.set_implicit_auxiliary_derivatives_weight(AuxDerivWeight);

guess = MocoTrajectory('.\test\example2DWalking\InitialGuessFileForTracking.sto');%solver.createGuess();

solver.setGuess(guess)


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
gaitTrackingSolution = study.solve();
reference_data = MocoTrajectory('.\output\track\outputReference\states_half.sto');


% Check to make sure the problem solved successfully
if ~strcmp(gaitTrackingSolution.getStatus(),"Solve_Succeeded")
    error('The problem did not solve succesfully - exiting');
end

% Extract some information from the solution
output(1) = gaitTrackingSolution.getObjective();
output(2) = gaitTrackingSolution.getSolverDuration();
output(3) = gaitTrackingSolution.getNumIterations();

% Save the solution with # of mesh intervals and processor cores in the filename

% Also save this solution with a generic filename that can be used as the
% initial guess on the next finest mesh density
gaitTrackingSolution.write(output_dir + 'states_half.sto');



% Optional output: Everything from here to the end is only for animation
% and plotting purposes and could be commented out or deleted.

% Create a full stride from the periodic single step solution
fullStride = opensimMoco.createPeriodicTrajectory(gaitTrackingSolution);
fullStride.write(output_dir + 'states.sto');

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

gaitTrackingSolution.write(output_dir + 'states_half.sto');
ref = MocoTrajectory(output_dir + '/outputReference/states_half.sto');

if gaitTrackingSolution.isNumericallyEqual(ref, 0.1)
    warning("output matches output reference for Gait Tracking");
else
    error("tracking failed to match reference output for goal");
end

end