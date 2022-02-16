/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoActivationSquaredGoal.cpp                                *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi                                                     *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoActivationSquaredGoal.h"

using namespace OpenSim;

void MocoActivationSquaredGoal::constructProperties() {
    constructProperty_divide_by_displacement(false);
}

void MocoActivationSquaredGoal::initializeOnModelImpl(const Model& model) const {
    auto allSysYIndices = createSystemYIndexMap(model);
    
    for (const auto& muscle : model.getComponentList<Muscle>()) {
        if (!muscle.get_ignore_activation_dynamics()) {
            const std::string path = muscle.getAbsolutePathString();
            int activationIndex = allSysYIndices[path + "/activation"];
            m_act_indices.push_back(activationIndex);
        }
    }
    setRequirements(1, 1);
}

void MocoActivationSquaredGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    getModel().realizeDynamics(input.state);
    const auto& states = input.state.getY();
    integrand = 0.0;
    for (int i = 0; i < (int)m_act_indices.size(); ++i) {
        integrand += SimTK::square(states[m_act_indices[i]]);
    }   
}

void MocoActivationSquaredGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;
    if (get_divide_by_displacement()) {
        cost[0] /=
            calcSystemDisplacement(input.initial_state, input.final_state);
    }
}
