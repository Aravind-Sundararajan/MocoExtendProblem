function test_ExtendProblem()
% Load the Moco libraries
opensimroot = 'C:\opensim 4.3\'; %create a char array that has the opensim path toplevel directory
addpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add the opensim path to the
javaaddpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add opensimroot bin and the java path to MATLAB's dynamic path path
setenv('PATH', [[opensimroot 'bin'] ';' [opensimroot 'sdk\lib'] ';' getenv('PATH')]);% Set Windows System path to include OpenSim libraries
import org.opensim.modeling.* %import opensim api library
opensimCommon.LoadOpenSimLibraryExact('C:\Users\asundar4\Desktop\MocoCustomGoal\build\RelWithDebInfo\osimMocoZMPGoal.dll');

% Create MocoStudy.
% ================
track = MocoTrack();
track.setName('gaitTracking');

ControlEffortWeight = 1;
StateTrackWeight = 0;
GRFTrackingWeight   = 0;

% Define the optimal control problem.
% ===================================
% Reference data for tracking problem
tableProcessor = TableProcessor('./input/torque/humanWalk_AvgIK.sto');
tableProcessor.append(TabOpLowPassFilter(6));

modelProcessor = ModelProcessor('./model/model_31d84m_modified_contacts.osim');
modelProcessor.append(ModOpIgnoreTendonCompliance());
%weld some of the joints
jointsToWeld = StdVectorString();
% Weld shoulder and elbow joints for initial torque-driven sim. 
jointsToWeld.add('shoulder_r');
jointsToWeld.add('shoulder_l');
jointsToWeld.add('elbow_r');
jointsToWeld.add('elbow_l');
%Weld arm, hand and foot
jointsToWeld.add('radioulnar_r');
jointsToWeld.add('radioulnar_l');
jointsToWeld.add('wrist_r');
jointsToWeld.add('wrist_l');
jointsToWeld.add('mtp_r');
jointsToWeld.add('mtp_l');
jointsToWeld.add('subtalar_r');
jointsToWeld.add('subtalar_l');
modelProcessor.append(ModOpReplaceJointsWithWelds(jointsToWeld));

track.setModel(modelProcessor);
track.setStatesReference(tableProcessor);
track.set_states_global_tracking_weight(StateTrackWeight);
track.set_allow_unused_references(true);
% track.set_track_reference_position_derivatives(true);
% track.set_apply_tracked_states_to_guess(true);
track.set_initial_time(0.0);
track.set_final_time(1.16);

%track.set_guess_file('./output/predictive_simulations/effort/1_00.sto');%'./output/best_so_far_qp.sto');

% Coordinate-specific tracking weights (value & speed) as SDs averaged over the
% gait cycle from O'Neill et al. 2021)
%                                                            
pq = StateTrackWeight; % Divided by 44 because there are 44 tracking targets: 32 DoF, 6 GRF components
coordWeights = MocoWeightSet();
coordWeights.cloneAndAppend(MocoWeight('/jointset/groundPelvis/pelvis_tx/value',          0));
coordWeights.cloneAndAppend(MocoWeight('/jointset/groundPelvis/pelvis_ty/value',          pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/groundPelvis/pelvis_tz/value',          pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/groundPelvis/pelvis_tilt/value',        pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/groundPelvis/pelvis_list/value',        pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/groundPelvis/pelvis_rotation/value',    pq));

coordWeights.cloneAndAppend(MocoWeight('/jointset/lumbar/lumbar_flex/value',               pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/lumbar/lumbar_add/value',                pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/lumbar/lumbar_rot/value',                pq));

coordWeights.cloneAndAppend(MocoWeight('/jointset/hip_r/hip_flexion_r/value',              pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/hip_r/hip_adduction_r/value',            pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/hip_r/hip_rotation_r/value',             pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/knee_r/knee_angle_r/value',              pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/ankle_r/ankle_angle_r/value',            pq));

coordWeights.cloneAndAppend(MocoWeight('/jointset/hip_l/hip_flexion_l/value',              pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/hip_l/hip_adduction_l/value',            pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/hip_l/hip_rotation_l/value',             pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/knee_l/knee_angle_l/value',              pq));
coordWeights.cloneAndAppend(MocoWeight('/jointset/ankle_l/ankle_angle_l/value',            pq));


pw = pq*0.0001; % Scale the generalized speed tracking errors by this constant
coordWeights.cloneAndAppend(MocoWeight('/jointset/groundPelvis/pelvis_tx/speed',          0));
coordWeights.cloneAndAppend(MocoWeight('/jointset/groundPelvis/pelvis_ty/speed',          pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/groundPelvis/pelvis_tz/speed',          pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/groundPelvis/pelvis_tilt/speed',        pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/groundPelvis/pelvis_list/speed',        pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/groundPelvis/pelvis_rotation/speed',    pw));

coordWeights.cloneAndAppend(MocoWeight('/jointset/lumbar/lumbar_flex/speed',               pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/lumbar/lumbar_add/speed',                pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/lumbar/lumbar_rot/speed',                pw));

coordWeights.cloneAndAppend(MocoWeight('/jointset/hip_r/hip_flexion_r/speed',              pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/hip_r/hip_adduction_r/speed',            pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/hip_r/hip_rotation_r/speed',             pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/knee_r/knee_angle_r/speed',              pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/ankle_r/ankle_angle_r/speed',            pw));

coordWeights.cloneAndAppend(MocoWeight('/jointset/hip_l/hip_flexion_l/speed',              pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/hip_l/hip_adduction_l/speed',            pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/hip_l/hip_rotation_l/speed',             pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/knee_l/knee_angle_l/speed',              pw));
coordWeights.cloneAndAppend(MocoWeight('/jointset/ankle_l/ankle_angle_l/speed',            pw));
track.set_states_weight_set(coordWeights);

study = track.initialize();
problem = study.updProblem();

% Goals
% =====

% Symmetry
symmetryGoal = MocoPeriodicityGoal('symmetryGoal');
problem.addGoal(symmetryGoal);
model = modelProcessor.process();
model.initSystem();

% Symmetric state values (except for pelvis_tx) and speeds
for i = 1:model.getNumStateVariables()
    currentStateName = string(model.getStateVariableNames().getitem(i-1));
    if (~contains(currentStateName,'pelvis_tx/value'))
       symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
    end
end

% Symmetric controls
for i = 1:model.getNumControls()
    currentControlName = string(problem.createRep().createControlInfoNames().get(i-1));
    %disp(currentControlName);
    if (~contains(currentControlName,'_pelvis_pelvis_tx'))
        symmetryGoal.addControlPair(MocoPeriodicityGoalPair(currentControlName));
    end
end

% Get a reference to the MocoControlGoal that is added to every MocoTrack
% problem by default and change the weight
effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));
effort.setWeight(ControlEffortWeight);
effort.setExponent(3);
effort.setDivideByDisplacement(false);

% Contact (GRF) tracking (if weight not set to zero)
if GRFTrackingWeight ~= 0
    contactTracking = MocoContactTrackingGoal('contact', GRFTrackingWeight);
    contactTracking.setExternalLoadsFile('./input/muscle/grf_humanwalk.xml');
    
    contactSpheres_r = StdVectorString();
    contactSpheres_l = StdVectorString();
    
    contactSpheres_r.add("contactHeel_r");
    contactSpheres_r.add("contactMedHeel_r");
    contactSpheres_r.add("contactLatHeel_r");
    contactSpheres_r.add("contactMid_r");
    contactSpheres_r.add("contactLatMid_r");
    contactSpheres_r.add("contactMedMid_r");
    contactSpheres_r.add("contactMH1_r");
    contactSpheres_r.add("contactMH3_r");
    contactSpheres_r.add("contactMH5_r");
    contactSpheres_r.add("contactHallux_r");
    contactSpheres_r.add("contactOtherToes_r");
    
    contactSpheres_l.add("contactHeel_l");
    contactSpheres_l.add("contactMedHeel_l");
    contactSpheres_l.add("contactLatHeel_l");
    contactSpheres_l.add("contactMid_l");
    contactSpheres_l.add("contactLatMid_l");
    contactSpheres_l.add("contactMedMid_l");
    contactSpheres_l.add("contactMH1_l");
    contactSpheres_l.add("contactMH3_l");
    contactSpheres_l.add("contactMH5_l");
    contactSpheres_l.add("contactHallux_l");
    contactSpheres_l.add("contactOtherToes_l");
    
    contactTracking.addContactGroup(contactSpheres_r,"Right_GRF");
    contactTracking.addContactGroup(contactSpheres_l,"Left_GRF");
    
    % Example code if there is a two part foot
    % altFramesRightFoot = StdVectorString(); % for two-part foot
    % altFramesRightFoot.add('/bodyset/r_toes');
    % contactTracking.addContactGroup(MocoContactTrackingGoalGroup( ...
    %               forceNamesRightFoot,'Right_GRF',altFramesRightFoot));
    
    problem.addGoal(contactTracking);
end

cptr = uint64(problem.getCPtr(problem));
ep = extend_problem(cptr);
ep.addCustomGoal(1);

% Bounds
% ======
problem.setStateInfo("/jointset/groundPelvis/pelvis_rotation/value", [-75*pi/180, 75*pi/180]);
problem.setStateInfo("/jointset/groundPelvis/pelvis_list/value", [-20*pi/180, 20*pi/180]);
problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value", [-30*pi/180, 30*pi/180]);

%problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", [-2, 2], -0.6555, [0.25, 1.0]);
problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", [0, 1.5],0,[1.0,1.35]);

problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value", [0.20, 1.5]);
problem.setStateInfo("/jointset/groundPelvis/pelvis_tz/value", [-0.50, 0.50]);
problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value", [-20*pi/180, 90*pi/180]);
problem.setStateInfo("/jointset/hip_r/hip_adduction_r/value", [-20*pi/180, 20*pi/180]);
problem.setStateInfo("/jointset/hip_r/hip_rotation_r/value", [-40*pi/180, 15*pi/180]);
problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value", [-20*pi/180, 90*pi/180]);
problem.setStateInfo("/jointset/hip_l/hip_adduction_l/value", [-20*pi/180, 20*pi/180]);
problem.setStateInfo("/jointset/hip_l/hip_rotation_l/value", [-40*pi/180, 15*pi/180]);
problem.setStateInfo("/jointset/knee_r/knee_angle_r/value", [-125*pi/180, 0]);
problem.setStateInfo("/jointset/knee_l/knee_angle_l/value", [-125*pi/180, 0]);
problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value", [-30*pi/180, 40*pi/180]);
problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value", [-30*pi/180, 40*pi/180]);
% problem.setStateInfo("/jointset/r_mtp/r_mtp_angle/value", [-75*pi/180, 50*pi/180]);
% problem.setStateInfo("/jointset/l_mtp/l_mtp_angle/value", [-75*pi/180, 50*pi/180]);
problem.setStateInfo("/jointset/lumbar/lumbar_flex/value", [-10*pi/180, 10*pi/180]);
problem.setStateInfo("/jointset/lumbar/lumbar_add/value", [-10*pi/180, 10*pi/180]);
problem.setStateInfo("/jointset/lumbar/lumbar_rot/value", [-10*pi/180, 10*pi/180]);

problem.setStateInfoPattern('/forceset/.*_r/activation',   [0.01, 1.0], [], []);
problem.setStateInfoPattern('/forceset/.*_l/activation',   [0.01, 1.0], [], []);
problem.setStateInfoPattern('/forceset/.*/normalized_tendon_force',   [0, 1.8], [], []);
problem.setStateInfoPattern('/forceset/lumbar_.*/activation',   [-1.0, 1.0], [], []);
problem.setStateInfoPattern('/forceset/reserve_.*/activation',   [-1.0, 1.0], [], []);
problem.setControlInfoPattern('.*_r', [0.01, 1.0], [], []);
problem.setControlInfoPattern('.*_l', [0.01, 1.0], [], []);
problem.setControlInfoPattern('/forceset/lumbar_.*', [-1.0, 1.0], [], []);
problem.setControlInfoPattern('/forceset/reserve_.*', [-1.0, 1.0], [], []);


myGoal = problem.updGoal("ZMP");

% Solve the problem
% =================
solver = MocoCasADiSolver.safeDownCast(study.updSolver());
solver.set_optim_max_iterations(5);
solver.set_num_mesh_intervals(50); % With Hermite-Simpson leads to 2n+1 grid points
solver.set_parallel(18);  % Set to # of actual cores on laptop
solver.set_optim_constraint_tolerance(1e-4);
solver.set_optim_convergence_tolerance(1e-2);

solver.resetProblem(problem);
guess = solver.createGuess();
lowerOrderSolution = MocoTrajectory('./output/predictive_simulations/effort/1_00.sto');
lowerOrderStatesTable = lowerOrderSolution.exportToStatesTable();
lowerOrderControlsTable = lowerOrderSolution.exportToControlsTable();
guess.insertStatesTrajectory(lowerOrderStatesTable, true);
guess.insertControlsTrajectory(lowerOrderControlsTable, true);

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
