/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoCustomOutputGoal.cpp                   *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                               *
 *                                                                            *
 * -------------------------------------------------------------------------- */
#include "MocoCustomOutputGoal.h"
#include "RegisterTypes_osimMocoCustomOutputGoal.h"

using namespace OpenSim;

static osimMocoCustomOutputGoalInstantiator instantiator;

OSIMMOCOCUSTOMOUTPUTGOAL_API void RegisterTypes_osimMocoCustomOutputGoal() {
    try {
        Object::registerType(MocoCustomOutputGoal());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMocoCustomOutputGoal "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoCustomOutputGoalInstantiator::osimMocoCustomOutputGoalInstantiator() {
    registerDllClasses();
}

void osimMocoCustomOutputGoalInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoCustomOutputGoal();
}
