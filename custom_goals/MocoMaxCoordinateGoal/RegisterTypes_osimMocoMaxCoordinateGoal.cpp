/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoMaxCoordinateGoal.cpp                  *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */
#include "MocoMaxCoordinateGoal.h"
#include "RegisterTypes_osimMocoMaxCoordinateGoal.h"

using namespace OpenSim;

static osimMocoMaxCoordinateGoalInstantiator instantiator;

OSIMMOCOMAXCOORDINATEGOAL_API void RegisterTypes_osimMocoMaxCoordinateGoal() {
    try {
        Object::registerType(MocoMaxCoordinateGoal());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMocoMaxCoordinateGoal "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoMaxCoordinateGoalInstantiator::osimMocoMaxCoordinateGoalInstantiator() {
    registerDllClasses();
}

void osimMocoMaxCoordinateGoalInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoMaxCoordinateGoal();
}
