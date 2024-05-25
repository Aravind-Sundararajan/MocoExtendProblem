function WalkSim_predictive45(sim_type)

addpath(genpath(fullfile(pwd,'bin','RelWithDebInfo'))); % Extend Problem (magic!)
mesh_int= 25;
cores = 1;
guess_strategy ='CG';
coarsest_mesh = 25;

output_dirs = ["./output/effpred/",...
    "./output/meppredmarkerAccel/",...
    "./output/meppredBOS/",...
    "./output/meppredZMP/",...    
    ];% 
AccelerationWeights = [ (1e-3)/11 (1e-3)/11 (1e-3)/11 (1e-3)/11]; %(1e-11)/4 (1e-11)/4 (1e-5)/2 (1e-11)/4 (1e-6)/4 ];
j = sim_type + 1;
output_dir =output_dirs(j); 
acceleration_weight=AccelerationWeights(j);
%------------------------------------------------------------------------
% Solve a tracking problem where the goal is to minimize the difference
% between simulated and reference coordinate values and speeds, and GRFs,
% as well as to minimize an effort cost (squared controls). The reference
% data are from Nguyen et al. (2019, IEEE TNSRE) and represent half a gait
% cycle. Symmetry of the gait cycle and a prescribed average walking speed
% are enforced through endpoint constraints.
% The number of temporal mesh intervals, the number of processor cores
% and the approach to use for the initial guess are all passed in by the
% calling script.
%
% Authors: Alex Denton & Brian Umberger
%


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

%track.setStatesReference(tableProcessor);
%track.set_states_global_tracking_weight(StateTrackingWeight);
%track.set_allow_unused_references(true);
%track.set_track_reference_position_derivatives(true);

% Set the initial guess based on whether we are using the consistent
% guess (CG) approach or the mesh refinement (MR) approach.
% See the publication for more details.


% Always use the provided guess file for the intial guess
%track.set_guess_file('.\output\track\states_half.sto');

% Adjust the weights so some specific coordinates are not tracked
% coordWeights = MocoWeightSet();
% coordWeights.cloneAndAppend(MocoWeight('/jointset/mtp_r/mtp_angle_r/value', 0));
% coordWeights.cloneAndAppend(MocoWeight('/jointset/mtp_l/mtp_angle_l/value', 0));
% coordWeights.cloneAndAppend(MocoWeight('/jointset/mtp_r/mtp_angle_r/speed', 0));
% coordWeights.cloneAndAppend(MocoWeight('/jointset/mtp_l/mtp_angle_l/speed', 0));
% coordWeights.cloneAndAppend(MocoWeight('/jointset/ground_pelvis/pelvis_tx/value', 0));
% coordWeights.cloneAndAppend(MocoWeight('/jointset/ground_pelvis/pelvis_tx/speed', 0));
% track.set_states_weight_set(coordWeights);

%study = track.initialize();
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
%effortGoal.setDivideByDisplacement(true);

left_foot = char(model.getBodySet().get("calcn_l").getAbsolutePathString());
right_foot = char(model.getBodySet().get("calcn_r").getAbsolutePathString());

cptr = uint64(problem.getCPtr(problem));
ep = extend_problem(cptr);


div_disp =false;
div_dur =false;
div_mass =false;

if j == 1
    %eff pred
elseif j ==2
    ep.addMocoMarkerAccelerationGoal('addMocoMarkerAccelerationGoal',10.0,...
        div_disp, div_dur, div_mass,...
        char(model.getMarkerSet().get("head_marker").getAbsolutePathString()));
elseif j ==3
    ep.addMocoBOSGoal('base_of_support',10.0,...
        div_disp, div_dur,div_mass, 1,...
        left_foot, right_foot);
elseif j ==4
    ep.addMocoZMPGoal("zero_moment_point",10.0,...
        div_disp, div_dur,div_mass, 1); 
end

% A custom goal for minimizing acceleration per distance. Uses an explicit
% formulation.
% Author: A. Sundarajan, MWU
% AccelerationWeight = AccelerationWeights(j);
% if AccelerationWeight ~= 0
% 	divide_by_displacement = true;
% 	cptr = uint64(problem.getCPtr(problem));
% 	ep3 = extend_problem(cptr);
% 	coords = [""];
% 	pelvis_coords = [""];
% 	for c = 1:model.getCoordinateSet().getSize()
% 		coords(c) = string(model.getCoordinateSet().get(c-1).getAbsolutePathString());
%     end
% 	disp("explicit accel goal limb coords:")
% 	disp(coords);
% 	ep3.addMocoCoordinateAccelerationGoal(...
% 		'coord_accel_limbs',...
% 		AccelerationWeight,...
% 		divide_by_displacement,...
% 		convertStringsToChars(coords));
% end

% Contact (GRF) tracking goal
% contactTracking = MocoContactTrackingGoal('contact',GRFTrackingWeight);
% contactTracking.setExternalLoadsFile('.\test\example2DWalking\WalkSimReferenceGRF.xml');

% forceNamesRightFoot = StdVectorString();
% forceNamesRightFoot.add("Contact_Foot_Ground_R1");
% forceNamesRightFoot.add("Contact_Foot_Ground_R2");
% forceNamesRightFoot.add("Contact_Foot_Ground_R3");
% forceNamesRightFoot.add("Contact_Foot_Ground_R4");
% forceNamesRightFoot.add("Contact_Foot_Ground_R5");
% forceNamesRightFoot.add("Contact_Foot_Ground_R6");
% forceNamesRightFoot.add("Contact_Foot_Ground_R7");
% forceNamesRightFoot.add("Contact_Foot_Ground_R8");
% altFramesRightFoot = StdVectorString();
% altFramesRightFoot.add('/bodyset/toes_r');
% contactTracking.addContactGroup(MocoContactTrackingGoalGroup(forceNamesRightFoot,'Right_GRF',altFramesRightFoot));
% 
% forceNamesLeftFoot = StdVectorString();
% forceNamesLeftFoot.add("Contact_Foot_Ground_L1");
% forceNamesLeftFoot.add("Contact_Foot_Ground_L2");
% forceNamesLeftFoot.add("Contact_Foot_Ground_L3");
% forceNamesLeftFoot.add("Contact_Foot_Ground_L4");
% forceNamesLeftFoot.add("Contact_Foot_Ground_L5");
% forceNamesLeftFoot.add("Contact_Foot_Ground_L6");
% forceNamesLeftFoot.add("Contact_Foot_Ground_L7");
% forceNamesLeftFoot.add("Contact_Foot_Ground_L8");
% altFramesLeftFoot = StdVectorString();
% altFramesLeftFoot.add('/bodyset/toes_l');
% contactTracking.addContactGroup(MocoContactTrackingGoalGroup(forceNamesLeftFoot,'Left_GRF',altFramesLeftFoot));
% 
% contactTracking.setProjection('plane');
% contactTracking.setProjectionVector(Vec3(0, 0, 1));
%problem.addGoal(contactTracking);


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

%solver.setGuess(gaitTrackingSolution);



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



% guess = solver.createGuess();
% guess_track = MocoTrajectory('.\output\track\states_half.sto');
% trackStatesTable = guess_track.exportToStatesTable();
% trackControlsTable = guess_track.exportToControlsTable();
% 
% 
% guess.insertStatesTrajectory(trackStatesTable, true);
% guess.insertControlsTrajectory(trackControlsTable, true);
% solver.setGuess(guess)


% Solve the problem
% =================
gaitPredictiveSolution = study.solve();
%reference_data = MocoTrajectory(output_dir + 'outputReference\states_tracked_states.sto');

% if ~gaitPredictiveSolution.isNumericallyEqual(reference_data)
%     error('The simulation is not numerically equal to outputReference. Check ' + output_dir);
% end

% Check to make sure the problem solved successfully
if ~strcmp(gaitPredictiveSolution.getStatus(),"Solve_Succeeded")
    error('The problem did not solve succesfully - exiting');
end

% Extract some information from the solution
output(1) = gaitPredictiveSolution.getObjective();
output(2) = gaitPredictiveSolution.getSolverDuration();
output(3) = gaitPredictiveSolution.getNumIterations();

% Save the solution with # of mesh intervals and processor cores in the filename
%gaitPredictiveSolution.write(output_dir + 'gaitTracking_solution_' + string(mesh_int) + 'i_' + string(cores) + 'c' + '.sto');

% Also save this solution with a generic filename that can be used as the
% initial guess on the next finest mesh density
gaitPredictiveSolution.write(output_dir + 'states_half.sto');



% Optional output: Everything from here to the end is only for animation
% and plotting purposes and could be commented out or deleted.

% Create a full stride from the periodic single step solution
fullStride = opensimMoco.createPeriodicTrajectory(gaitPredictiveSolution);
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

ref = MocoTrajectory(output_dir + '/outputReference/states_half.sto');

if gaitPredictiveSolution.isNumericallyEqual(ref)
    warning("output matches output reference for Gait Tracking");
else
    warning("tracking failed to match reference output for goal");
end

end
