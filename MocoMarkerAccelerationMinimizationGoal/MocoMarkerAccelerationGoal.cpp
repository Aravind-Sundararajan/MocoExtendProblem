/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoMarkerAccelerationGoal.cpp                               *
 * Author(s): Varun Joshi                                                     *
 * -------------------------------------------------------------------------- */

#include "MocoMarkerAccelerationGoal.h"

using namespace OpenSim;
using SimTK::Vec3;

void MocoMarkerAccelerationGoal::initializeOnModelImpl(
        const Model& model) const {
    const auto& marker_name = getMarkerName();
    const auto& markerSet = model.getMarkerSet();
    int iset = -1;
    for (int i = 0; i < (int)markRefNames.size(); ++i) {
        if (model.hasComponent<Marker>(marker_name)) {
            m_model_marker = model.getComponent<Marker>(markRefNames[i]);
        } else {
            OPENSIM_THROW_FRMOBJ(Exception,
                    "Marker '{}' unrecognized by the specified model.",
                    marker_name);
            }
        }
    setRequirements(1, 1);
}

void MocoMarkerAccelerationGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
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
                 m_model_markers->getLinearAccelerationInGround(input.state);

    // Find the error magnitudes
    for (int i = 0; i < 3; ++i){
        integrand += sqrt(SimTK::square(markerAcceleration[i]) + eps);
    }
}

void MocoMarkerAccelerationGoal::calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral;
}

void MocoMarkerAccelerationGoal::printDescriptionImpl() const {
    log_cout("Minimizing accelerations for marker name: {}", get_marker_name());
}