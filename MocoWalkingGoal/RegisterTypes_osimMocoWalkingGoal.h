#ifndef OPENSIM_REGISTERTYPES_OSIMMOCOWALKING_H
#define OPENSIM_REGISTERTYPES_OSIMMOCOWALKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: RegisterTypes_osimMocoWalkingGoal.h                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi                                                     *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoWalkingGoalDLL.h"

extern "C" {

OSIMMOCOWALKINGGOAL_API void RegisterTypes_osimMocoWalkingGoal();

}

class osimMocoWalkingGoalInstantiator {
public:
    osimMocoWalkingGoalInstantiator();
private:
    void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIMMOCOWALKINGGOAL_H
