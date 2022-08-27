#ifndef OPENSIM_REGISTERTYPES_OSIMMOCOOUTPUTTRACKING_H
#define OPENSIM_REGISTERTYPES_OSIMMOCOOUTPUTTRACKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: RegisterTypes_osimMocoOutputTrackingGoal.h                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi                                                     *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoOutputTrackingGoalDLL.h"

extern "C"
{

    OSIMMOCOOUTPUTTRACKINGGOAL_API void RegisterTypes_osimMocoOutputTrackingGoal();
}

class osimMocoOutputTrackingGoalInstantiator
{
public:
    osimMocoOutputTrackingGoalInstantiator();

private:
    void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIMMOCOOUTPUTTRACKINGGOAL_H
