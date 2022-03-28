/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoMarkerAccelerationGoal.cpp             *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi                                                     *
 *                                                                            *
 * -------------------------------------------------------------------------- */
#include "MocoMarkerAccelerationGoal.h"
#include "RegisterTypes_osimMocoMarkerAccelerationGoal.h"

using namespace OpenSim;

static osimMocoMarkerAccelerationGoalInstantiator instantiator;

OSIMMOCOMAXCOORDINATEGOAL_API void RegisterTypes_osimMocoMarkerAccelerationGoal() {
    try {
        Object::registerType(MocoMarkerAccelerationGoal());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMocoMarkerAccelerationGoal "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoMarkerAccelerationGoalInstantiator::osimMocoMarkerAccelerationGoalInstantiator() {
    registerDllClasses();
}

void osimMocoMarkerAccelerationGoalInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoMarkerAccelerationGoal();
}
