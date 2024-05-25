/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCoordinateAccelerationGoal.cpp                           *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            * 
 *                                                                            *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoCoordinateAccelerationGoal.h"

using namespace OpenSim;

void MocoCoordinateAccelerationGoal::constructProperties() {
    constructProperty_exponent(2);
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
      int exponent = get_exponent();

  // The pow() function gives slightly different results than x * x. On Mac,
  // using x * x requires fewer solver iterations.
  if (exponent == 1) {
    m_power_function = [](const double &x) { return std::abs(x); };
  } else if (exponent == 2) {
    m_power_function = [](const double &x) { return x * x; };
  } else {
    m_power_function = [exponent](const double &x) {
      return pow(std::abs(x), exponent);
    };
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
}