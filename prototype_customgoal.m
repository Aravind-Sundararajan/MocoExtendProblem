function value = examplePrototypeCustomGoal

opensimroot = 'C:\opensim 4.3\'; %create a char array that has the opensim path toplevel directory
addpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add the opensim path to the
javaaddpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add opensimroot bin and the java path to MATLAB's dynamic path path
setenv('PATH', [[opensimroot 'bin'] ';' [opensimroot 'sdk\lib'] ';' getenv('PATH')]);% Set Windows System path to include OpenSim libraries
import org.opensim.modeling.* %import opensim api library

study = MocoStudy();
problem = study.updProblem();

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
model = modelProcessor.process();

% Add metabolic cost model
metabolics = Bhargava2004SmoothedMuscleMetabolics();
metabolics.setName('metabolic_cost');
metabolics.set_use_smoothing(true);

ms = model.getMuscles();
for m = 0: ms.getSize()-1
    this_muscle_name = ms.get(m).getName();
    this_muscle      = ms.get(m);
    metabolics.addMuscle(this_muscle_name, this_muscle);
end

model.addComponent(metabolics);
model.finalizeConnections();

problem.setModel(model);
problem.setTimeBounds(0, 1.16);

solver = study.initCasADiSolver();
solver.resetProblem(problem);
trajectory = solver.createGuess();
lowerOrderSolution = MocoTrajectory('./output/1_00.sto');
lowerOrderStatesTable = lowerOrderSolution.exportToStatesTable();
lowerOrderControlsTable = lowerOrderSolution.exportToControlsTable();
trajectory.insertStatesTrajectory(lowerOrderStatesTable, true);
trajectory.insertControlsTrajectory(lowerOrderControlsTable, true);
n_muscles = 76;
n_coordinates = 21;
n_passive = 14;
totalMass = 75.3370;

w = [((500/n_muscles)/totalMass) 2000/n_muscles 50000/n_coordinates 1000/n_passive];
value = evaluateCustomGoal(problem, trajectory,w, ...
        @calcMyCustomEffortGoalIntegrand, @calcMyCustomEffortGoalValue);

end

function integrand = calcMyCustomEffortGoalIntegrand(model, state,w)

import org.opensim.modeling.* %import opensim api library

% Compute the integrand for the integral portion of the cost goal.

model.realizeAcceleration(state);
met = Bhargava2004SmoothedMuscleMetabolics.safeDownCast(model.getComponent('metabolic_cost'));
ms = model.getMuscles();
n_muscles = ms.getSize();

for m = 0:model.getMuscles.getSize()-1
    this_muscle = model.getMuscles().get(m);
    metabolic_cost(m+1) = met.getMuscleMetabolicRate(state,this_muscle.getAbsolutePathString());
end

% This computes the sum of squared element values (squared 2-norm).
integrand = 0;
for i = 0:n_muscles - 1
    integrand = integrand + w(1)*metabolic_cost(i+1)^2;
end

controls = model.getControls(state);
for i = 0:controls.size() - 1
    integrand = integrand + w(2)*controls.get(i)^2;
end

udots = state.getUDot();
for i = 0:udots.size() - 1
    integrand = integrand + w(3)*udots.get(i)^2;
end

fs = model.getForceSet();
for f = 0:fs.getSize()-1
    this_force = fs.get(f);
    if contains(string(this_force.getName()),'Passive','IgnoreCase',true)
        exp = ExpressionBasedCoordinateForce.safeDownCast(this_force);
        integrand = integrand + w(4)*exp.getForceMagnitude(state)^2;
    end
end
disp(integrand);

end

function value = calcMyCustomEffortGoalValue(...
                        model, initial_state, final_state, integral)
% Compute the goal value from the phase's initial and final states, and from
% the integral of the integrand function over the phase. The integration is
% performed for you by evaluateCustomGoal().
value = integral / (final_state.getTime() - initial_state.getTime());
end

function goalValue = evaluateCustomGoal(...
                        problem, mocoTraj,w, integrandFunc, goalFunc)
% Test a custom goal, defined by integrand and goal functions, on the
% provided problem and MocoTrajectory.
model = problem.getPhase(0).getModelProcessor().process();
org.opensim.modeling.opensimMoco.prescribeControlsToModel(mocoTraj, model);
statesTraj = mocoTraj.exportToStatesTrajectory(problem);
model.initSystem();
N = statesTraj.getSize();
integrand = zeros(N, 1);
for i = 0:(N - 1)
    integrand(i + 1) = integrandFunc(model, statesTraj.get(i),w);
end

integral = trapz(integrand);
goalValue = goalFunc(model, statesTraj.front(), statesTraj.back(), integral);

end

