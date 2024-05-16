/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoMuscleStrainGoal.cpp                                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                               *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoMuscleStrainGoal.h"
#include <OpenSim/Actuators/ModelOperators.h>
using namespace OpenSim;
#define tolerance std::numeric_limits<float>::epsilon()

void MocoMuscleStrainGoal::constructProperties() {
  constructProperty_divide_by_displacement(false);
  constructProperty_exponent(2);
}

void MocoMuscleStrainGoal::initializeOnModelImpl(const Model &model) const {
  
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

            m_Fiso.push_back(muscle.getMaxIsometricForce());
            m_Lopt.push_back(muscle.getOptimalFiberLength());
            m_Vol.push_back(muscle.getMaxIsometricForce() * muscle.getOptimalFiberLength());
        }
    }
    
  int exponent = get_exponent();

  // The pow() function gives slightly different results than x * x. On Mac,
  // using x * x requires fewer solver iterations.
  if (exponent == 1) {
    m_power_function = [](const double &x) { return x; };
  } else if (exponent == 2) {
    m_power_function = [](const double &x) { return x * x; };
  } else {
    m_power_function = [exponent](const double &x) {
      return pow(std::abs(x), exponent);
    };
  }
    setRequirements(1, 1);
}

void MocoMuscleStrainGoal::calcIntegrandImpl(const IntegrandInput &input,
                                             double &integrand) const {
  integrand = 0.0;

  getModel().realizeAcceleration(input.state);
  
  //proxy for muscle volume is Fiso * Lopt
  //divide proxyV for realized fiber length and Actuation

 // Read the states
    const auto& states = input.state.getY();
    
    for (const auto& f : getModel().getComponentList<OpenSim::Force>()) {
	    if (f.getConcreteClassName() == "DeGrooteFregly2016Muscle"){
            auto& fc = dynamic_cast<const OpenSim::DeGrooteFregly2016Muscle&>(f);
        integrand += m_power_function(
            (fc.getActuation(input.state) * fc.getFiberLength(input.state)) /
            (fc.getMaxIsometricForce() * fc.getOptimalFiberLength())
            );
			//integrand += m_power_function((fc.getActuation(input.state) * fc.getFiberLength(input.state)));
        }
    }
}

void MocoMuscleStrainGoal::calcGoalImpl(const GoalInput &input,
                                        SimTK::Vector &cost) const {
  cost[0] = input.integral;
  // Divide by the displacement if divide_by_displacement property is true
  if (get_divide_by_displacement()) {
    cost[0] /= calcSystemDisplacement(input.initial_state, input.final_state);
  }
}

SimTK::Matrix
MocoMuscleStrainGoal::FlattenSpatialVec(const SimTK::SpatialVec &S) const {
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
