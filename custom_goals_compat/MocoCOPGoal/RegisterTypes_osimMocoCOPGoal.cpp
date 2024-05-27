/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoCOPGoal.cpp              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */
#include "MocoCOPGoal.h"
#include "RegisterTypes_osimMocoCOPGoal.h"

using namespace OpenSim;

static osimMocoCOPGoalInstantiator instantiator;

OSIMMOCOCOPGOAL_API void RegisterTypes_osimMocoCOPGoal() {
    try {
        Object::registerType(MocoCOPGoal());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMocoCOPGoal "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoCOPGoalInstantiator::osimMocoCOPGoalInstantiator() {
    registerDllClasses();
}

void osimMocoCOPGoalInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoCOPGoal();
}
