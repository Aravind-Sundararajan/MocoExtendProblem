/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoOutputTrackingGoal.cpp                 *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */
#include "MocoOutputTrackingGoal.h"
#include "RegisterTypes_osimMocoOutputTrackingGoal.h"

using namespace OpenSim;

static osimMocoOutputTrackingGoalInstantiator instantiator;

OSIMMOCOOUTPUTTRACKINGGOAL_API void RegisterTypes_osimMocoOutputTrackingGoal()
{
    try
    {
        Object::registerType(MocoOutputTrackingGoal());
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR during osimMocoOutputTrackingGoal "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoOutputTrackingGoalInstantiator::osimMocoOutputTrackingGoalInstantiator()
{
    registerDllClasses();
}

void osimMocoOutputTrackingGoalInstantiator::registerDllClasses()
{
    RegisterTypes_osimMocoOutputTrackingGoal();
}
