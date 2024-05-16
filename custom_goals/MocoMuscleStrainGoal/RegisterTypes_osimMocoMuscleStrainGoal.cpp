/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoMuscleStrainGoal.cpp              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi *
 *                                                                            *
 * -------------------------------------------------------------------------- */
#include "MocoMuscleStrainGoal.h"
#include "RegisterTypes_osimMocoMuscleStrainGoal.h"

using namespace OpenSim;

static osimMocoMuscleStrainGoalInstantiator instantiator;

OSIMMOCOMUSCLESTRAINGOAL_API void RegisterTypes_osimMocoMuscleStrainGoal() {
  try {
    Object::registerType(MocoMuscleStrainGoal());
  } catch (const std::exception &e) {
    std::cerr << "ERROR during osimMocoMuscleStrainGoal "
                 "Object registration:\n"
              << e.what() << std::endl;
  }
}

osimMocoMuscleStrainGoalInstantiator::osimMocoMuscleStrainGoalInstantiator() {
  registerDllClasses();
}

void osimMocoMuscleStrainGoalInstantiator::registerDllClasses() {
  RegisterTypes_osimMocoMuscleStrainGoal();
}
