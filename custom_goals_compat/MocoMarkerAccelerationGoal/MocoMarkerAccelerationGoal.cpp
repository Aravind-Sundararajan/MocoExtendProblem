/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoMarkerAccelerationGoal.cpp             *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi, Aravind Sundararajan                                                      *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoMarkerAccelerationGoal.h"

using namespace OpenSim;
using SimTK::Vec3;


void MocoMarkerAccelerationGoal::constructProperties() {
    constructProperty_marker_name("/markerSet/testMarker");
    constructProperty_exponent(1);
    constructProperty_divide_by_displacement(false);
    

}

void MocoMarkerAccelerationGoal::initializeOnModelImpl(
        const Model& model) const {
    // Get the marker
    m_model_marker.reset(&model.getComponent<Point>(get_marker_name()));
    setRequirements(1, 1);
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
        integrand += SimTK::square(markerAcceleration[i]);
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