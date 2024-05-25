/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoBOSGoal.cpp                                              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                               *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoBOSGoal.h"
#include <OpenSim/Actuators/ModelOperators.h>
using namespace OpenSim;
using namespace SimTK;
#define tolerance std::numeric_limits<float>::epsilon()

void MocoBOSGoal::constructProperties() {
  	constructProperty_exponent(2);
    constructProperty_left_foot_frame("");
    constructProperty_right_foot_frame("");
}

void MocoBOSGoal::initializeOnModelImpl(const Model& model) const {
    setRequirements(1, 1);
    
	m_left_foot_frame = getModel().getComponent<Body>(get_left_foot_frame());
    m_right_foot_frame = getModel().getComponent<Body>(get_right_foot_frame());

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

void MocoBOSGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
	integrand = 0.0;
	comData out;
    getModel().realizeAcceleration(input.state);
	
	SimTK::Vec3 base_of_support(0.0);

    SimTK::Vec3 v_r = m_left_foot_frame->getMassCenter();
    SimTK::Vec3 v_l = m_right_foot_frame->getMassCenter();

    SimTK::Vec3 v_r_com_g = m_right_foot_frame->findStationLocationInGround(input.state, v_r);
    SimTK::Vec3 v_l_com_g = m_left_foot_frame->findStationLocationInGround(input.state,  v_l);
    

	SimTK::Vec3 mass_center = getModel().calcMassCenterPosition(input.state);

    mass_center[1] = 0.0;
    v_r[1] = 0.0;
    v_l[1] = 0.0;
    

    base_of_support =  mass_center - avg(v_l_com_g, v_r_com_g);// avg the 2 feet

    integrand += m_power_function(base_of_support.norm());
}

void MocoBOSGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;
}

SimTK::Matrix MocoBOSGoal::FlattenSpatialVec(const SimTK::SpatialVec& S) const {
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

SimTK::Vec3 MocoBOSGoal::avg(const SimTK::Vec3& a, const SimTK::Vec3& b) const {
    SimTK::Vec3 out(0.0);
    out[0] = 0.5*(a[0] + b[0]);
    out[1] = 0.5*(a[1] + b[1]);
    out[2] = 0.5*(a[2] + b[2]);
    return out;
}

