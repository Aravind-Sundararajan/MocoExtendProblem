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
    StatesTrajectory st = StatesTrajectory();
    std::vector<double> inte;
}

void MocoMaxCoordinateGoal::initializeOnModelImpl(const Model& model) const {
    setRequirements(1, 1);
    auto& coordName = m_state_name;
    auto& coord = getModel().getCoordinateSet().get(coordName);
    auto allSysYIndices = createSystemYIndexMap(model);
    auto refName = getModel().getCoordinateSet().get(coordName).getAbsolutePathString();               // Get current state variable name
    refName = refName + "/value";                                // Add on value so that we find accelerations using UDot
    if (allSysYIndices.count(refName) != 0) {                    // Find state variable in system map
        m_state_index = allSysYIndices[refName];                 // If found, store index of state variable
    }
    coord_max = coord.getRangeMax();
    std::cout << "coord max of " << refName <<" is " << coord_max << "." << std::endl;
}

void MocoMaxCoordinateGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    const auto& state = input.state;
    getModel().realizeDynamics(input.state);

    double d = SimTK::square(state.getQ().get(m_state_index) - coord_max);

    if ((integrand >= d) || (integrand <= 0.0)){
        integrand = d;
    }
}

void MocoMaxCoordinateGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = std::sqrt(input.integral);
    if (get_divide_by_displacement()) {
        cost[0] /=
            calcSystemDisplacement(input.initial_state, input.final_state);
    }
}