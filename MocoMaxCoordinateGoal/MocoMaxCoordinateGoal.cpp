/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoMaxCoordinateGoal.cpp                                              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 * this uses the LSE                                                          *
 * (a smooth approx. of the maximum function that is C2 continuous)           *
 * -------------------------------------------------------------------------- */

#include "MocoMaxCoordinateGoal.h"

using namespace OpenSim;

void MocoMaxCoordinateGoal::constructProperties() {
    constructProperty_divide_by_displacement(false);
}

void MocoMaxCoordinateGoal::initializeOnModelImpl(const Model& model) const {
    setRequirements(1, 1);
}

void MocoMaxCoordinateGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    getModel().realizeAcceleration(input.state);
    const auto& state = input.state;
    auto udots = state.getUDot();
	for (int i = 0; i < udots.size(); i++) {
		integrand += SimTK::square(udots.get(i));
	} 
}

void MocoMaxCoordinateGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;
    if (get_divide_by_displacement()) {
        cost[0] /=
            calcSystemDisplacement(input.initial_state, input.final_state);
    }
}