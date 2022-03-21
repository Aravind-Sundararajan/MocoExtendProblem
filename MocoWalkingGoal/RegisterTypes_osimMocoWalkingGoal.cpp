/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoWalkingGoal.cpp              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */
#include "MocoWalkingGoal.h"
#include "RegisterTypes_osimMocoWalkingGoal.h"

using namespace OpenSim;

static osimMocoWalkingGoalInstantiator instantiator;

OSIMMOCOWALKINGGOAL_API void RegisterTypes_osimMocoWalkingGoal() {
    try {
        Object::registerType(MocoWalkingGoal());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMocoWalkingGoal "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoWalkingGoalInstantiator::osimMocoWalkingGoalInstantiator() {
    registerDllClasses();
}

void osimMocoWalkingGoalInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoWalkingGoal();
}
