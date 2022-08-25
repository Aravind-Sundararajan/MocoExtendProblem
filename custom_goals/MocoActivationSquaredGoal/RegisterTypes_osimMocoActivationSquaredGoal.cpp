/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoActivationSquaredGoal.cpp              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi                                                     *
 *                                                                            *
 * -------------------------------------------------------------------------- */
#include "MocoActivationSquaredGoal.h"
#include "RegisterTypes_osimMocoActivationSquaredGoal.h"

using namespace OpenSim;

static osimMocoActivationSquaredGoalInstantiator instantiator;

OSIMMOCOACTIVATIONSQUAREDGOAL_API void RegisterTypes_osimMocoActivationSquaredGoal() {
    try {
        Object::registerType(MocoActivationSquaredGoal());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMocoActivationSquaredGoal "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoActivationSquaredGoalInstantiator::osimMocoActivationSquaredGoalInstantiator() {
    registerDllClasses();
}

void osimMocoActivationSquaredGoalInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoActivationSquaredGoal();
}
