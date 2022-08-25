/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoMarkerAccelerationGoal.cpp                               *
 * Author(s): Varun Joshi                                                     *
 * -------------------------------------------------------------------------- */

#include "MocoMarkerAccelerationGoal.h"

using namespace OpenSim;
using SimTK::Vec3;


void MocoMarkerAccelerationGoal::constructProperties() {
    constructProperty_marker_name("");
    constructProperty_divide_by_displacement(false);
}

void MocoMarkerAccelerationGoal::initializeOnModelImpl(
        const Model& model) const {
    // Get the marker
    m_model_marker.reset(&model.getComponent<Point>(get_marker_name()));
    setRequirements(1, 1);
}

void MocoMarkerAccelerationGoal::calcIntegrandImpl(
        const IntegrandInput& input, SimTK::Real& integrand) const {
    // Set the integrand value
    integrand = 0;

    // Set constant for smoothing of absolute values
    const double eps = 1e-16;

    // Read the state and the time
    const auto& state = input.state;
    const auto& time = state.getTime();

    // Realize acceleration to get the acceleration values
    getModel().realizeAcceleration(state);

    // Compute marker acceleration
    const auto& markerAcceleration =
                 m_model_marker->getAccelerationInGround(state);

    // Find the error magnitudes
    for (int i = 0; i < 3; ++i){
        integrand += sqrt(SimTK::square(markerAcceleration[i]) + eps);
    }
}

void MocoMarkerAccelerationGoal::calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const {
        cost[0] = input.integral;

        // Divide the cost by displacement
        if (get_divide_by_displacement()) {
            cost[0] /= calcSystemDisplacement(input.initial_state, input.final_state);
        }
}