function test_ExtendProblem()
% Load the Moco libraries
opensimroot = 'C:\opensim 4.3\'; %create a char array that has the opensim path toplevel directory
addpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add the opensim path to the
javaaddpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add opensimroot bin and the java path to MATLAB's dynamic path path
setenv('PATH', [[opensimroot 'bin'] ';' [opensimroot 'sdk\lib'] ';' getenv('PATH')]);% Set Windows System path to include OpenSim libraries
import org.opensim.modeling.* %import opensim api library
opensimCommon.LoadOpenSimLibraryExact('C:\Users\oneill_lab\Desktop\MocoExtendProblem\build\RelWithDebInfo\osimMocoActivationSquaredGoal.dll');

% Create MocoStudy.
% ================
track = MocoTrack();
track.setName('gaitTracking');

controlEffortWeight = 0;
stateTrackingWeight = 1;
GRFTrackingWeight   = 1;

% Define the optimal control problem.
% ===================================
% Reference data for tracking problem
tableProcessor = TableProcessor('./input/referenceCoordinates.sto');
tableProcessor.append(TabOpLowPassFilter(6));

modelProcessor = ModelProcessor('./model/2D_gait.osim');
track.setModel(modelProcessor);
track.setStatesReference(tableProcessor);
track.set_states_global_tracking_weight(stateTrackingWeight);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);
track.set_initial_time(0.0);
track.set_final_time(0.47008941);
study = track.initialize();
problem = study.updProblem();

% Goals
% =====

% Symmetry
% --------
% This goal allows us to simulate only one step with left-right symmetry
% that we can then double to create a full gait cycle.
symmetryGoal = MocoPeriodicityGoal('symmetryGoal');
problem.addGoal(symmetryGoal);
model = modelProcessor.process();
model.initSystem();

% Symmetric coordinate values (except for pelvis_tx) and speeds. Here, we 
% constrain final coordinate values of one leg to match the initial value of the 
% other leg. Or, in the case of the pelvis_tx value, we constrain the final 
% value to be the same as the initial value.
for i = 1:model.getNumStateVariables()
    currentStateName = string(model.getStateVariableNames().getitem(i-1));
    if startsWith(currentStateName , '/jointset')
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
        if (~contains(currentStateName,'_r') && ...
            ~contains(currentStateName,'_l') && ...
            ~contains(currentStateName,'pelvis_tx/value') && ...
            ~contains(currentStateName,'/activation'))
            symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
        end
    end
end

% Symmetric muscle activations. Here, we constrain final muscle activation 
% values of one leg to match the initial activation values of the other leg.
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

% The lumbar coordinate actuator control is symmetric.
symmetryGoal.addControlPair(MocoPeriodicityGoalPair('/forceset/lumbarAct'));

% Get a reference to the MocoControlGoal that is added to every MocoTrack
% problem by default and change the weight
effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));
effort.setWeight(controlEffortWeight);

% Optionally, add a contact tracking goal.
if GRFTrackingWeight ~= 0
    % Track the right and left vertical and fore-aft ground reaction forces.
    contactTracking = MocoContactTrackingGoal('contact', GRFTrackingWeight);
    contactTracking.setExternalLoadsFile('./input/referenceGRF.xml');
    forceNamesRightFoot = StdVectorString();
    forceNamesRightFoot.add('contactHeel_r');
    forceNamesRightFoot.add('contactFront_r');
    contactTracking.addContactGroup(forceNamesRightFoot, 'Right_GRF');
    forceNamesLeftFoot = StdVectorString();
    forceNamesLeftFoot.add('contactHeel_l');
    forceNamesLeftFoot.add('contactFront_l');
    contactTracking.addContactGroup(forceNamesLeftFoot, 'Left_GRF');
    contactTracking.setProjection('plane');
    contactTracking.setProjectionVector(Vec3(0, 0, 1));
    problem.addGoal(contactTracking);
end

cptr = uint64(problem.getCPtr(problem));
ep = extend_problem(cptr);
ep.addCustomGoal(1);

% Bounds
% ======
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


myGoal = problem.updGoal("ActivationCost");

% Solve the problem
% =================
solver = MocoCasADiSolver.safeDownCast(study.updSolver());
solver.set_optim_max_iterations(5);
solver.set_num_mesh_intervals(50); % With Hermite-Simpson leads to 2n+1 grid points
solver.set_parallel(18);  % Set to # of actual cores on laptop
solver.set_optim_constraint_tolerance(1e-4);
solver.set_optim_convergence_tolerance(1e-2);

gaitTrackingSolution = study.solve();
gaitTrackingSolution.unseal();
gaitTrackingSolution.write('./output/halfstride.sto')
% Create a full stride from the periodic single step solution.
% For details, view the Doxygen documentation for createPeriodicTrajectory().
fullStride = opensimMoco.createPeriodicTrajectory(gaitTrackingSolution);
fullStride.write('./output/gaitTracking_solution_fullStride.sto');

% Extract ground reaction forces
% ==============================
contact_r = StdVectorString();
contact_l = StdVectorString();
contact_r.add('contactHeel_r');
contact_r.add('contactFront_r');
contact_l.add('contactHeel_l');
contact_l.add('contactFront_l');

externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(model, ...
                                 fullStride,contact_r,contact_l);
STOFileAdapter.write(externalForcesTableFlat, ...
                             './output/gaitTracking_solutionGRF_fullStride.sto');
                         
ep.delete();
end
