/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCoordinateAccelerationGoal.cpp                                              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 * This goal minimizes coordinate accelerations for explicit multibody dynamic*
 * -------------------------------------------------------------------------- */

#include "MocoCoordinateAccelerationGoal.h"

using namespace OpenSim;

void MocoCoordinateAccelerationGoal::constructProperties() {
    constructProperty_divide_by_displacement(false);
}

void MocoCoordinateAccelerationGoal::initializeOnModelImpl(const Model& model) const {
    setRequirements(1, 1);
}

void MocoCoordinateAccelerationGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    getModel().realizeAcceleration(input.state);
    const auto& state = input.state;
    auto udots = state.getUDot();
	for (int i = 0; i < udots.size(); i++) {
		integrand += SimTK::square(udots.get(i));
	} 
}

void MocoCoordinateAccelerationGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;
    if (get_divide_by_displacement()) {
        cost[0] /=
            calcSystemDisplacement(input.initial_state, input.final_state);
    }
}
