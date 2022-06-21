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
    auto allSysYIndices = createSystemYIndexMap(model);

    for (int i = 0; i < (int)m_state_names.size(); ++i) {
        auto& refName = m_state_names[i];           // Get current state variable name
        refName = refName + "/value";               // Add on value so that we find accelerations using UDot
        if (allSysYIndices.count(refName) == 0) {   // Find state variable in system map
            continue;                               // If not found, skip the current variable
        } else {
            m_state_indices.push_back(allSysYIndices[refName]); // If found, store index of state variable
            std::cout << "Found " << refName << " = " << allSysYIndices[refName] << std::endl;
        }
    }

    std::cout << "Number of states to minimize: " << (int)m_state_indices.size() << std::endl;
}

void MocoCoordinateAccelerationGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    getModel().realizeAcceleration(input.state);
    const auto& state = input.state;   
    auto udots = state.getUDot();
	for (int i = 0; i < udots.size(); i++) {
        if ((std::find(m_state_indices.begin(), m_state_indices.end(), i) != m_state_indices.end())) {
            integrand += SimTK::square(udots.get(i));
        }
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
