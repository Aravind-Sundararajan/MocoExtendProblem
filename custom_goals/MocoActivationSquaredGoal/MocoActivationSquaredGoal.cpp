/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoActivationSquaredGoal.cpp                                *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi, Aravind Sundararajan                               *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoActivationSquaredGoal.h"

using namespace OpenSim;

// Set all the properties to their default values
void MocoActivationSquaredGoal::constructProperties() {
    constructProperty_divide_by_displacement(false);
    constructProperty_end_point_goal(0.0);
}

void MocoActivationSquaredGoal::initializeOnModelImpl(const Model& model) const {
    // Make a map to get indices corresponding to every state in the model
    auto allSysYIndices = createSystemYIndexMap(model);
    
    // Loop through all the muscles
    for (const auto& muscle : model.getComponentList<Muscle>()) {
        // Make sure activation dynamics aren't being ignored for the current muscle
        if (!muscle.get_ignore_activation_dynamics()) {

            // Find the path to the activation for this muscle and get the correspoding state index
            const std::string path = muscle.getAbsolutePathString();
            int activationIndex = allSysYIndices[path + "/activation"];

            // Store the index of the activation state
            m_act_indices.push_back(activationIndex);
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
    // Set the requirements for the integrator - 1 input, 1 output
    setRequirements(1, 1);
}

void MocoActivationSquaredGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    // Initialize the integrand
    integrand = 0.0;

    getModel().realizeDynamics(input.state);

    // Read the states
    const auto& states = input.state.getY();
    
    // For each muscle, find the square of the activation and add it to the integrand
    for (int i = 0; i < (int)m_act_indices.size(); ++i) {
        integrand += SimTK::square(states[m_act_indices[i]]);
    }   
}

void MocoActivationSquaredGoal::calcGoalImpl(
    const GoalInput& input, SimTK::Vector& cost) const {

    // Cost mode
    cost[0] = input.integral;

    // Divide by the displacement if divide_by_displacement property is true
    if (get_divide_by_displacement()) {
        cost[0] /=
            calcSystemDisplacement(input.initial_state, input.final_state);
    }

    // End-point constraint mode
    if (!getModeIsCost()) {
        cost[0] -= get_end_point_goal();
    }
}

