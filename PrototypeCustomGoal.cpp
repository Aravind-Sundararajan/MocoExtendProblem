#include <OpenSim/Actuators/ActivationCoordinateActuator.h>
#include <OpenSim/Moco/osimMoco.h>

using namespace OpenSim;
using SimTK::Pi;

typedef double (*integrand_func)(const Model&, const SimTK::State&, const double);
typedef double (*goal_func)(const Model&, const SimTK::State&, const SimTK::State&, const double&);
integrand_func integrandFunc;
goal_func goalFunc;

std::unique_ptr<Model> createSlidingMassModel() {
	auto model = make_unique<Model>();
	model->setName("sliding_mass");
	model->set_gravity(SimTK::Vec3(0, 0, 0));
	auto* body = new Body("body", 10.0, SimTK::Vec3(0), SimTK::Inertia(0));
	model->addComponent(body);

	// Allows translation along x.
	auto* joint = new SliderJoint("slider", model->getGround(), *body);
	auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
	coord.setName("position");
	model->addComponent(joint);

	auto* actu = new CoordinateActuator();
	actu->setCoordinate(&coord);
	actu->setName("actuator");
	actu->setOptimalForce(1);
	model->addComponent(actu);

	body->attachGeometry(new Sphere(0.05));

	model->finalizeConnections();

	return model;
}

double trapz(SimTK::Vector& integrand) {
	int N = integrand.size() - 1;
	double step = (N) / (2.0 * ((double)N));
	SimTK::Vector v(integrand);
	for (int i = 1; i < N; i++) {
		v[i] *= 2.0;
	}
	double integral = v.sum();
	integral *= step;
	return integral;
}


double CustomGoalIntegrand(const Model& model, const SimTK::State& state, double w) {
	model.realizeAcceleration(state);
	double integrand = 0;
	auto udots = state.getUDot();
	for (int i = 0; i < udots.size(); i++) {
		integrand += w * pow(udots.get(i),2);
	}
	return integrand;
}

double CustomGoalValue(const Model& model, const SimTK::State& initial_state, const SimTK::State& final_state, const double& integral) {
	double value = integral / (final_state.getTime() - initial_state.getTime());
	return value;
}

double evaluateCustomGoal(const MocoProblem& problem, const MocoTrajectory& mocoTraj, integrand_func integrandFunc, goal_func goalFunc, const double w) {
	//     // Test a custom goal, defined by integrand and goal functions, on the
	//     // provided problem and MocoTrajectory.
	auto& model = problem.getPhase(0).getModelProcessor().process();
	prescribeControlsToModel(mocoTraj, model);
	auto& statesTraj = mocoTraj.exportToStatesTrajectory(problem);
	model.initSystem();
	int N = (int)statesTraj.getSize();
	SimTK::Vector integrand(N);
	for (int i = 0; i < N; i++) {
		integrand[i] = integrandFunc(model, statesTraj.get(i), w);
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

	MocoStudy study;
	MocoProblem& problem = study.updProblem();

	problem.setModel(createSlidingMassModel());
	// Bounds.
	// -------
	// Initial time must be 0, final time can be within [0, 1].
	problem.setTimeBounds(MocoInitialBounds(0.), MocoFinalBounds(1));

	// Position must be within [-5, 5] throughout the motion.
	// Initial position must be 0, final position must be 1.

	// Position must be within [-5, 5] throughout the motion.
	// Initial position must be 0, final position must be 1.
	problem.setStateInfo("/slider/position/value", MocoBounds(-1, 1), MocoInitialBounds(0), MocoFinalBounds(1));
	// Speed must be within [-50, 50] throughout the motion.
	// Initial and final speed must be 0. Use compact syntax.
	problem.setStateInfo("/slider/position/speed", 1, 1, 1);

	// Applied force must be between -50 and 50.
	problem.setControlInfo("/actuator", MocoBounds(-50, 50));

	// Solve the problem.
	// -----
	MocoCasADiSolver& solver = study.initCasADiSolver();
	solver.set_num_mesh_intervals(50);
	MocoSolution solution = study.solve();
	//solution.write("point_mass_solution.sto");

	solver = study.initCasADiSolver();
	solver.resetProblem(problem);

	integrandFunc = CustomGoalIntegrand;
	goalFunc = CustomGoalValue;
	std::cout << "Running custom goal on solution: " << std::endl;
	double w = 1.0;
	double myIntegral = evaluateCustomGoal(problem, solution, integrandFunc, goalFunc,w);
	std::cout << "custom goal value: " << myIntegral << std::endl;
	return EXIT_SUCCESS;
}
