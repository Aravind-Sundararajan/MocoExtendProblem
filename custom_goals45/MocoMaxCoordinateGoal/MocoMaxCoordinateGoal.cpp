/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoMaxCoordinateGoal.cpp                                              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoMaxCoordinateGoal.h"

using namespace OpenSim;

void MocoMaxCoordinateGoal::constructProperties() {
    StatesTrajectory st = StatesTrajectory();
    std::vector<double> inte;
    constructProperty_exponent(2);
//     double v = std::numeric_limits<double>::max();
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
}