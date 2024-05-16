/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoZMPGoal.cpp              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */
#include "MocoZMPGoal.h"
#include "RegisterTypes_osimMocoZMPGoal.h"

using namespace OpenSim;

static osimMocoZMPGoalInstantiator instantiator;

OSIMMOCOZMPGOAL_API void RegisterTypes_osimMocoZMPGoal() {
    try {
        Object::registerType(MocoZMPGoal());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMocoZMPGoal "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoZMPGoalInstantiator::osimMocoZMPGoalInstantiator() {
    registerDllClasses();
}

void osimMocoZMPGoalInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoZMPGoal();
}
