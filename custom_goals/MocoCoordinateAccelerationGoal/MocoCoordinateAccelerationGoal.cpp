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

  const std::string suffix = "/value";

  for (const auto& pair : allSysYIndices) {
    const std::string& path = pair.first;
    const int index = pair.second;
    if (path.length() >= suffix.length() && 
      path.compare(path.length() - suffix.length(), suffix.length(), suffix) == 0) {
      m_state_indices.push_back(index);
    }
  }

  for (int i = 0; i < (int)m_custom_state_names.size(); ++i) {
    auto& refName = m_custom_state_names[i];
    auto& refWeight = m_custom_weights_input[i];
    refName = refName + "/value";
    if (allSysYIndices.count(refName) == 0) {
      continue;
    } else {
      m_custom_acc_indices.push_back(allSysYIndices[refName]);
      m_custom_weights.push_back(refWeight);
    }
  }

  int exponent = get_exponent();

  // The pow() function gives slightly different results than x * x. On Mac,
  // using x * x requires fewer solver iterations.
  if (exponent == 1) {
    m_power_function = [](const double &x) { 
      return std::sqrt(x*x + 1e-8); 
    };
  } else if (exponent == 2) {
    m_power_function = [](const double &x) { 
      return x * x; 
    };
  } else {
    m_power_function = [exponent](const double &x) {
      return pow(std::abs(x), exponent);
    };
  }
}

void MocoCoordinateAccelerationGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {

  integrand = 0.0;

  getModel().realizeAcceleration(input.state);

  const auto& udots = input.state.getUDot();

	for (int i = 0; i < udots.size(); i++) {
    auto it = std::find(m_custom_acc_indices.begin(), m_custom_acc_indices.end(), m_state_indices[i]);
    if (it != m_custom_acc_indices.end()) {
        // Get the index position in m_custom_act_indices
        size_t custom_idx = std::distance(m_custom_acc_indices.begin(), it);
        integrand += m_custom_weights[custom_idx] * m_power_function(udots[m_state_indices[i]]);
    } else {
        integrand += m_power_function(udots[m_state_indices[i]]);
    }
	}
}

void MocoCoordinateAccelerationGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;
}