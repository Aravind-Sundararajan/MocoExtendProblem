/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoActivationGoal.cpp                                *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi, Aravind Sundararajan                                                      *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoActivationGoal.h"

using namespace OpenSim;

// Set all the properties to their default values
void MocoActivationGoal::constructProperties() {
    constructProperty_end_point_goal(0.0);
    constructProperty_exponent(2);
}

void MocoActivationGoal::initializeOnModelImpl(const Model& model) const {
    // Set the requirements for the integrator - 1 input, 1 output
    setRequirements(1, 1);
    
    // Make a map to get indices corresponding to every state in the model
    auto allSysYIndices = createSystemYIndexMap(model);
    
    const std::string suffix = "/activation";

    for (const auto& pair : allSysYIndices) {
        const std::string& path = pair.first;
        const int index = pair.second;
        if (path.length() >= suffix.length() && 
            path.compare(path.length() - suffix.length(), suffix.length(), suffix) == 0) {
            m_act_indices.push_back(index);
        }
    }

    for (int i = 0; i < (int)m_custom_state_names.size(); ++i) {
      //std::cout << "Custom state name: " << m_custom_state_names[i] << std::endl;
        auto& refName = m_custom_state_names[i];
        auto& refWeight = m_custom_weights_input[i];
        refName = refName + "/activation";
        if (allSysYIndices.count(refName) == 0) {
            //std::cout << "Custom state name not found: " << refName << std::endl;
            continue;
        } else {
            //std::cout << "Custom state name found: " << refName << std::endl;
            m_custom_act_indices.push_back(allSysYIndices[refName]);
            //std::cout << "Custom act index: " << allSysYIndices[refName] << std::endl;
            m_custom_weights.push_back(refWeight);
            //std::cout << "Custom weight: " << refWeight << std::endl;
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
    
}

void MocoActivationGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    // Initialize the integrand
    integrand = 0.0;

    getModel().realizeDynamics(input.state);

    // Read the states
    const auto& states = input.state.getY();
    
    // For each muscle, find the square of the activation and add it to the integrand
    for (int i = 0; i < (int)m_act_indices.size(); ++i) {
        auto it = std::find(m_custom_act_indices.begin(), m_custom_act_indices.end(), m_act_indices[i]);
        if (it != m_custom_act_indices.end()) {
            // Get the index position in m_custom_act_indices
            size_t custom_idx = std::distance(m_custom_act_indices.begin(), it);
            integrand += m_custom_weights[custom_idx] * m_power_function(states[m_act_indices[i]]);
            //std::cout << "Custom weight found: " << m_custom_weights[custom_idx] << std::endl;
        } else {
            //std::cout << "Custom weight not found: " << m_act_indices[i] << std::endl;
            integrand += m_power_function(states[m_act_indices[i]]);
        }
    } 
}

void MocoActivationGoal::calcGoalImpl(
    const GoalInput& input, SimTK::Vector& cost) const {

    // Cost mode
    cost[0] = input.integral;

    // End-point constraint mode
    if (!getModeIsCost()) {
        cost[0] -= get_end_point_goal();
    }
}

