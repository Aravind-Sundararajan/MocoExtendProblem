% Load the Moco libraries
import org.opensim.modeling.*;
% Define the optimal control problem
% ==================================
study = MocoStudy();
study.setName('gaitPrediction');

problem = study.updProblem();
modelProcessor = ModelProcessor('2D_gait.osim');
problem.setModelProcessor(modelProcessor);


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
            pair = MocoPeriodicityGoalPair(currentStateName, ...
                           regexprep(currentStateName,'_r','_l') );
            symmetryGoal.addStatePair(pair);
        end
        if contains(currentStateName,'_l')
            pair = MocoPeriodicityGoalPair(currentStateName, ...
                           regexprep(currentStateName,'_l','_r'));
            symmetryGoal.addStatePair(pair);
        end
        if (~contains(currentStateName,'_r') && ...
            ~contains(currentStateName,'_l') && ...
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
            pair = MocoPeriodicityGoalPair(currentStateName, ...
                           regexprep(currentStateName,'_r','_l'));
            symmetryGoal.addStatePair(pair);
        end
        if contains(currentStateName,'_l')
            pair = MocoPeriodicityGoalPair(currentStateName, ...
                           regexprep(currentStateName,'_l','_r'));
            symmetryGoal.addStatePair(pair);
        end
    end
end

% Symmetric coordinate actuator controls
symmetryGoal.addControlPair(MocoPeriodicityGoalPair('/lumbarAct'));


% Prescribed average gait speed
speedGoal = MocoAverageSpeedGoal('speed');
problem.addGoal(speedGoal);
speedGoal.set_desired_average_speed(1.2);

% Effort over distance
effortGoal = MocoControlGoal('effort', 1.0);
problem.addGoal(effortGoal);
effortGoal.setExponent(3);
effortGoal.setDivideByDisplacement(true);

left_foot = char(model.getBodySet().get("calcn_l").getAbsolutePathString());
right_foot = char(model.getBodySet().get("calcn_r").getAbsolutePathString());

%MocoExtendProblem
cptr = uint64(problem.getCPtr(problem));
ep = extend_problem(cptr);
%ep.addMocoMuscleStrainGoal('MuscleVolume', 1.0, 2, true);
%ep.addMocoBOSGoal('base_of_support', 1.0, 2, true, left_foot, right_foot);
ep.addMocoZMPGoal('zero_moment_point', 10.0, 2, true);
% ep.addMocoMarkerAccelerationGoal('marker_acceleration', 1.0, ...
%     char(model.getMarkerSet().get("head_marker").getAbsolutePathString()), ...
%     true);

% Bounds
% ======
problem.setTimeBounds(0, [0.4, 0.6]);
problem.setStateInfo('/jointset/groundPelvis/pelvis_tilt/value', [-20*pi/180, -10*pi/180]);
problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0, 1]);
problem.setStateInfo('/jointset/groundPelvis/pelvis_ty/value', [0.75, 1.25]);
problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value', [-10*pi/180, 60*pi/180]);
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-10*pi/180, 60*pi/180]);
problem.setStateInfo('/jointset/knee_l/knee_angle_l/value', [-50*pi/180, 0]);
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [-50*pi/180, 0]);
problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value', [-15*pi/180, 25*pi/180]);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', [-15*pi/180, 25*pi/180]);
problem.setStateInfo('/jointset/lumbar/lumbar/value', [0, 20*pi/180]);


% Configure the solver
% ====================
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(50);
solver.set_verbosity(2);
solver.set_parallel(1);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1e-4);
solver.set_optim_constraint_tolerance(1e-4);
solver.set_optim_max_iterations(10000);
solver.set_parallel(1);
solver.setGuess(gaitTrackingSolution); % Use tracking solution as initial guess

% Solve problem
% =============
gaitPredictionSolution = study.solve();

% Create a full stride from the periodic single step solution.
% For details, view the Doxygen documentation for createPeriodicTrajectory().
fullStride = opensimMoco.createPeriodicTrajectory(gaitPredictionSolution);
fullStride.write('./output/meppredZMP/states.sto');

% Visualize the result.
%study.visualize(fullStride);

% Extract ground reaction forces
% ==============================
contact_r = StdVectorString();
contact_l = StdVectorString();
contact_r.add('contactHeel_r');
contact_r.add('contactFront_r');
contact_l.add('contactHeel_l');
contact_l.add('contactFront_l');

% Create a conventional ground reaction forces file by summing the contact
% forces of contact spheres on each foot.
% For details, view the Doxygen documentation for
% createExternalLoadsTableForGait().
externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(model, ...
                             fullStride, contact_r, contact_l);
STOFileAdapter.write(externalForcesTableFlat, ...
                             './output/meppredZMP/grf.sto');