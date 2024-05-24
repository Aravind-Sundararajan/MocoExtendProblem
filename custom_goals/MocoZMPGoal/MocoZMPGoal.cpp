/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoZMPGoal.cpp                                              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoZMPGoal.h"
#include <OpenSim/Actuators/ModelOperators.h>
using namespace OpenSim;
#define tolerance std::numeric_limits<float>::epsilon()

void MocoZMPGoal::constructProperties() {
  constructProperty_divide_by_displacement(false);
  constructProperty_exponent(1);
}

void MocoZMPGoal::initializeOnModelImpl(const Model &model) const {
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

void MocoZMPGoal::calcIntegrandImpl(const IntegrandInput &input,
                                    double &integrand) const {
  integrand = 0.0;

  getModel().realizeAcceleration(input.state);



    SimTK::Vec3 mass_center_position = getModel().calcMassCenterPosition(input.state);
    SimTK::Vec3 mass_center_velocity = getModel().calcMassCenterVelocity(input.state);
    SimTK::Vec3 mass_center_acceleration = getModel().calcMassCenterAcceleration(input.state);
    double h = mass_center_position[1];
    SimTK::Vec3 G = getModel().getGravity();
    double g  = G[1];
    SimTK::Vec3 zmp = SimTK::Vec3();
    //SimTK::Vec3 xCoM = SimTK::Vec3();
    //xCoM = mass_center_position + (-1.0*sqrt(h/g)) * (mass_center_velocity);
    //xCoM
    zmp = mass_center_position  + (-1.0 * h/g ) * (mass_center_acceleration);
    
    integrand += m_power_function((zmp - mass_center_position).norm());
}


void MocoZMPGoal::calcGoalImpl(const GoalInput &input,
                               SimTK::Vector &cost) const {
  cost[0] = input.integral;
  // Divide by the displacement if divide_by_displacement property is true
  if (get_divide_by_displacement()) {
    cost[0] /= calcSystemDisplacement(input.initial_state, input.final_state);
  }
}

SimTK::Matrix MocoZMPGoal::FlattenSpatialVec(const SimTK::SpatialVec &S) const {
  // turn a spatialvec into a 6x1 matrix.
  SimTK::Matrix spatialVecFlat(6, 1, 0.0);
  spatialVecFlat[0] = S[0][0];
  spatialVecFlat[1] = S[0][1];
  spatialVecFlat[2] = S[0][2];
  spatialVecFlat[3] = S[1][0];
  spatialVecFlat[4] = S[1][1];
  spatialVecFlat[5] = S[1][2];
  return spatialVecFlat;
}
