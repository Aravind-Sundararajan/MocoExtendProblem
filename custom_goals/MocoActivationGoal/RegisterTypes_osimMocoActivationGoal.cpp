/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoActivationGoal.cpp              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */
#include "MocoActivationGoal.h"
#include "RegisterTypes_osimMocoActivationGoal.h"

using namespace OpenSim;

static osimMocoActivationGoalInstantiator instantiator;

OSIMMOCOACTIVATIONGOAL_API void RegisterTypes_osimMocoActivationGoal() {
    try {
        Object::registerType(MocoActivationGoal());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMocoActivationGoal "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoActivationGoalInstantiator::osimMocoActivationGoalInstantiator() {
    registerDllClasses();
}

void osimMocoActivationGoalInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoActivationGoal();
}
