/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoBOSGoal.cpp                            *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                               *
 *                                                                            *
 * -------------------------------------------------------------------------- */
#include "MocoBOSGoal.h"
#include "RegisterTypes_osimMocoBOSGoal.h"

using namespace OpenSim;

static osimMocoBOSGoalInstantiator instantiator;

OSIMMOCOBOSGOAL_API void RegisterTypes_osimMocoBOSGoal() {
    try {
        Object::registerType(MocoBOSGoal());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMocoBOSGoal "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoBOSGoalInstantiator::osimMocoBOSGoalInstantiator() {
    registerDllClasses();
}

void osimMocoBOSGoalInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoBOSGoal();
}
