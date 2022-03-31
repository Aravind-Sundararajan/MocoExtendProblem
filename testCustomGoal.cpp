#include <string.h>

#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include "./MocoMaxCoordinateGoal/MocoMaxCoordinateGoal.h"
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Moco/MocoGoal/MocoOutputGoal.h>

using namespace OpenSim;
using SimTK::Pi;

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

int main()
{
	MocoStudy study;
	MocoProblem& problem = study.updProblem();

	problem.setModel(createSlidingMassModel());
	// Bounds.
	// -------
	// Initial time must be 0, final time can be within [0, 1].
	problem.setTimeBounds(MocoInitialBounds(0.), MocoFinalBounds(1.));

	// Position must be within [-5, 5] throughout the motion.
	// Initial position must be 0, final position must be 1.

	// Position must be within [-5, 5] throughout the motion.
	// Initial position must be 0, final position must be 1.
	problem.setStateInfo("/slider/position/value", MocoBounds(-1., 1.), MocoInitialBounds(0.), MocoFinalBounds(1.));
	// Speed must be within [-50, 50] throughout the motion.
	// Initial and final speed must be 0. Use compact syntax.
	problem.setStateInfo("/slider/position/speed", MocoBounds(-50, 50), MocoInitialBounds(0.), MocoFinalBounds(0.));

	// Applied force must be between -50 and 50.
	problem.setControlInfo("/actuator", MocoBounds(-50, 50));
	
	//activation squared goal
	problem.addGoal<MocoMaxCoordinateGoal>("Max",1000.0);
	
	// Solve the problem.
	// -----
	MocoCasADiSolver& solver = study.initCasADiSolver();
	solver.set_num_mesh_intervals(50);
	MocoSolution solution = study.solve();
	solution.write("point_mass_solution.sto");
	std::cout << "wrote to point_mass_solution.sto" << std::endl;

	solver = study.initCasADiSolver();
	solver.resetProblem(problem);
}
