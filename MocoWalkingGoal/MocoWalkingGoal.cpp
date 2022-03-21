/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoWalkingGoal.cpp                                              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoWalkingGoal.h"

using namespace OpenSim;

void MocoWalkingGoal::constructProperties() {
    constructProperty_divide_by_displacement(false);
}

void MocoWalkingGoal::initializeOnModelImpl(const Model& model) const {
    setRequirements(1, 1);
}

void MocoWalkingGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    getModel().realizeDynamics(input.state);
    const auto& state = input.state;
	const auto& m_model = getModel();

    integrand = 0.0;
    integrand += 1.0;  
}

void MocoWalkingGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;
    if (get_divide_by_displacement()) {
        cost[0] /=
            calcSystemDisplacement(input.initial_state, input.final_state);
    }
}
