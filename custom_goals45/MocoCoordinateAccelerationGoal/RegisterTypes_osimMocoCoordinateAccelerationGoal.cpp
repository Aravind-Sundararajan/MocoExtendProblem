/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoCoordinateAccelerationGoal.cpp              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */
#include "MocoCoordinateAccelerationGoal.h"
#include "RegisterTypes_osimMocoCoordinateAccelerationGoal.h"

using namespace OpenSim;

static osimMocoCoordinateAccelerationGoalInstantiator instantiator;

OSIMMOCOCOORDINATEACCELERATIONGOAL_API void RegisterTypes_osimMocoCoordinateAccelerationGoal() {
    try {
        Object::registerType(MocoCoordinateAccelerationGoal());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMocoCoordinateAccelerationGoal "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoCoordinateAccelerationGoalInstantiator::osimMocoCoordinateAccelerationGoalInstantiator() {
    registerDllClasses();
}

void osimMocoCoordinateAccelerationGoalInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoCoordinateAccelerationGoal();
}
