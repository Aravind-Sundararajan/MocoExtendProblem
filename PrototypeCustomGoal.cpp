#include <OpenSim/Actuators/ActivationCoordinateActuator.h>
#include <OpenSim/Moco/osimMoco.h>

using namespace OpenSim;
using SimTK::Pi;

typedef double (*integrand_func)(const Model&, const SimTK::State&);
typedef double (*goal_func)(const Model&, const SimTK::State&, const SimTK::State&, const double&);
integrand_func integrandFunc;
goal_func goalFunc;

double trapz(SimTK::Vector& integrand) {
	std::cout << integrand << std::endl;
	int N = integrand.size() - 1;
	double step = (N) / (2.0 * ((double)N));
	SimTK::Vector v(integrand);
	for (size_t i = 1; i < N; i++) {
		v[i] *= 2.0;
	}
	double integral = v.sum();
	integral *= step;
	return integral;
}


double CustomGoalIntegrand(const Model& model, const SimTK::State& state) {
	return 0.0;
}

double CustomGoalValue(const Model& model, const SimTK::State& initial_state, const SimTK::State& final_state, const double& integral) {
	return 0.0;
}

double evaluateCustomGoal(const MocoProblem& problem, const MocoTrajectory& mocoTraj, integrand_func integrandFunc, goal_func goalFunc) {
	//     // Test a custom goal, defined by integrand and goal functions, on the
	//     // provided problem and MocoTrajectory.
	auto& model = problem.getPhase(0).getModelProcessor().process();
	prescribeControlsToModel(mocoTraj, model);
	auto& statesTraj = mocoTraj.exportToStatesTrajectory(problem);
	model.initSystem();
	int N = statesTraj.getSize();
	SimTK::Vector integrand(N);
	for (size_t i = 0; i < N; i++) {
		integrand[i] = integrandFunc(model, statesTraj.get(i));
	}

	const double& integral = trapz(integrand);
	double goalValue = goalFunc(model, statesTraj.front(), statesTraj.back(), integral);
	return goalValue;
}

int main()
{
	double controlEffortWeight = 10;
	double stateTrackingWeight = 1;
	double GRFTrackingWeight = 1;

	MocoTrack track;
	track.setName("gaitTracking");

	ModelProcessor modelprocessor("C:/Users/oneill_lab/Desktop/MocoExtendProblem/models/2D_gait.osim");
	track.setModel(modelprocessor);
	track.setStatesReference(
		TableProcessor("C:/Users/oneill_lab/Desktop/MocoExtendProblem/input/referenceCoordinates.sto") | TabOpLowPassFilter(6));
	track.set_states_global_tracking_weight(stateTrackingWeight);
	track.set_allow_unused_references(true);
	track.set_track_reference_position_derivatives(true);
	track.set_apply_tracked_states_to_guess(true);
	track.set_initial_time(0.0);
	track.set_final_time(0.47008941);
	MocoStudy study = track.initialize();
	MocoProblem& problem = study.updProblem();

	//// Goals.
	//// =====
	//// Symmetry.
	//auto* symmetryGoal = problem.addGoal<MocoPeriodicityGoal>("symmetryGoal");
	//Model model = modelprocessor.process();
	//model.initSystem();
	//// Symmetric coordinate values (except for pelvis_tx) and speeds.
	//for (const auto& coord : model.getComponentList<Coordinate>()) {
	//	if (IO::EndsWith(coord.getName(), "_r")) {
	//		symmetryGoal->addStatePair({ coord.getStateVariableNames()[0],
	//				std::regex_replace(coord.getStateVariableNames()[0],
	//						std::regex("_r"), "_l") });
	//		symmetryGoal->addStatePair({ coord.getStateVariableNames()[1],
	//				std::regex_replace(coord.getStateVariableNames()[1],
	//						std::regex("_r"), "_l") });
	//	}
	//	if (IO::EndsWith(coord.getName(), "_l")) {
	//		symmetryGoal->addStatePair({ coord.getStateVariableNames()[0],
	//				std::regex_replace(coord.getStateVariableNames()[0],
	//						std::regex("_l"), "_r") });
	//		symmetryGoal->addStatePair({ coord.getStateVariableNames()[1],
	//				std::regex_replace(coord.getStateVariableNames()[1],
	//						std::regex("_l"), "_r") });
	//	}
	//	if (!IO::EndsWith(coord.getName(), "_l") &&
	//		!IO::EndsWith(coord.getName(), "_r") &&
	//		!IO::EndsWith(coord.getName(), "_tx")) {
	//		symmetryGoal->addStatePair({ coord.getStateVariableNames()[0],
	//				coord.getStateVariableNames()[0] });
	//		symmetryGoal->addStatePair({ coord.getStateVariableNames()[1],
	//				coord.getStateVariableNames()[1] });
	//	}
	//}
	//symmetryGoal->addStatePair({ "/jointset/groundPelvis/pelvis_tx/speed" });
	//// Symmetric coordinate actuator controls.
	//symmetryGoal->addControlPair({ "/lumbarAct" });
	//// Symmetric muscle activations.
	//for (const auto& muscle : model.getComponentList<Muscle>()) {
	//	if (IO::EndsWith(muscle.getName(), "_r")) {
	//		symmetryGoal->addStatePair({ muscle.getStateVariableNames()[0],
	//				std::regex_replace(muscle.getStateVariableNames()[0],
	//						std::regex("_r"), "_l") });
	//	}
	//	if (IO::EndsWith(muscle.getName(), "_l")) {
	//		symmetryGoal->addStatePair({ muscle.getStateVariableNames()[0],
	//				std::regex_replace(muscle.getStateVariableNames()[0],
	//						std::regex("_l"), "_r") });
	//	}
	//}
	//// Effort. Get a reference to the MocoControlGoal that is added to every
	//// MocoTrack problem by default.
	//MocoControlGoal& effort =
	//	dynamic_cast<MocoControlGoal&>(problem.updGoal("control_effort"));
	//effort.setWeight(controlEffortWeight);

	//// Optionally, add a contact tracking goal.
	//if (GRFTrackingWeight != 0) {
	//	// Track the right and left vertical and fore-aft ground reaction forces.
	//	auto* contactTracking = problem.addGoal<MocoContactTrackingGoal>(
	//		"contact", GRFTrackingWeight);
	//	contactTracking->setExternalLoadsFile("C:/Users/oneill_lab/Desktop/MocoExtendProblem/input/referenceGRF.xml");
	//	contactTracking->addContactGroup(
	//		{ "contactHeel_r", "contactFront_r" }, "Right_GRF");
	//	contactTracking->addContactGroup(
	//		{ "contactHeel_l", "contactFront_l" }, "Left_GRF");
	//	// Project the error onto the plane perpendicular to the +Z vector.
	//	contactTracking->setProjection("plane");
	//	contactTracking->setProjectionVector(SimTK::Vec3(0, 0, 1));
	//}

	//// Bounds.
	//// =======
	//problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
	//	{ -20 * Pi / 180, -10 * Pi / 180 });
	//problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", { 0, 1 });
	//problem.setStateInfo(
	//	"/jointset/groundPelvis/pelvis_ty/value", { 0.75, 1.25 });
	//problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
	//	{ -10 * Pi / 180, 60 * Pi / 180 });
	//problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
	//	{ -10 * Pi / 180, 60 * Pi / 180 });
	//problem.setStateInfo(
	//	"/jointset/knee_l/knee_angle_l/value", { -50 * Pi / 180, 0 });
	//problem.setStateInfo(
	//	"/jointset/knee_r/knee_angle_r/value", { -50 * Pi / 180, 0 });
	//problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
	//	{ -15 * Pi / 180, 25 * Pi / 180 });
	//problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
	//	{ -15 * Pi / 180, 25 * Pi / 180 });
	//problem.setStateInfo("/jointset/lumbar/lumbar/value", { 0, 20 * Pi / 180 });

	//// Configure the solver.
	//// =====================
	//MocoCasADiSolver& solver = study.updSolver<MocoCasADiSolver>();
	//solver.set_num_mesh_intervals(50);
	//solver.set_verbosity(2);
	//solver.set_optim_solver("ipopt");
	//solver.set_optim_convergence_tolerance(1e-4);
	//solver.set_optim_constraint_tolerance(1e-4);
	//solver.set_optim_max_iterations(1000);

	// Solve problem.
	// ==============
	MocoSolution solution = study.solve();
	auto full = createPeriodicTrajectory(solution);
	full.write("gaitTracking_solution_fullcycle.sto");

	//integrandFunc = CustomGoalIntegrand;
	//goalFunc = CustomGoalValue;
	//double myIntegral = evaluateCustomGoal(problem, full, integrandFunc, goalFunc);
	//std::cout << "custom goal value: " << myIntegral << std::endl;
	return EXIT_SUCCESS;
}
