function value = examplePrototypeCustomGoal

opensimroot = 'C:\opensim 4.3\'; %create a char array that has the opensim path toplevel directory
addpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add the opensim path to the
javaaddpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add opensimroot bin and the java path to MATLAB's dynamic path path
setenv('PATH', [[opensimroot 'bin'] ';' [opensimroot 'sdk\lib'] ';' getenv('PATH')]);% Set Windows System path to include OpenSim libraries
import org.opensim.modeling.* %import opensim api library

study = MocoStudy();
problem = study.updProblem();

modelProcessor = ModelProcessor('pointmass.osim');
model = modelProcessor.process();

problem.setModel(model);
% Bounds.
% -------
% Initial time must be 0, final time can be within [0, 5].
problem.setTimeBounds(MocoInitialBounds(0.), MocoFinalBounds(1));

% Position must be within [-5, 5] throughout the motion.
% Initial position must be 0, final position must be 1.

% Position must be within [-5, 5] throughout the motion.
% Initial position must be 0, final position must be 1.
problem.setStateInfo('/slider/position/value', MocoBounds(-1, 1), ...
    MocoInitialBounds(0), MocoFinalBounds(1));
% Speed must be within [-50, 50] throughout the motion.
% Initial and final speed must be 0. Use compact syntax.
problem.setStateInfo('/slider/position/speed', 1, 1, 1);

% Applied force must be between -50 and 50.
problem.setControlInfo('/actuator', MocoBounds(-50, 50));

% Cost.
% -----
%problem.addGoal(MocoFinalTimeGoal('final_time'));
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(50);
solution = study.solve();
solution.write('point_mass_solution.sto');

solver = study.initCasADiSolver();
solver.resetProblem(problem);

w = [1];
value = evaluateCustomGoal(problem, solution,w, ...
        @calcMyCustomEffortGoalIntegrand, @calcMyCustomEffortGoalValue);

end

function integrand = calcMyCustomEffortGoalIntegrand(model, state,w)

import org.opensim.modeling.* %import opensim api library
model.realizeAcceleration(state);
% Compute the integrand for the integral portion of the cost goal.
% This computes the sum of squared element values (squared 2-norm).
integrand = 0;
udots = state.getUDot();
for i = 0:udots.size() - 1
    integrand = integrand + w*udots.get(i)^2;
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

