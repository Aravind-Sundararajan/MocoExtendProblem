#ifndef OPENSIM_REGISTERTYPES_OSIMMOCOCOP_H
#define OPENSIM_REGISTERTYPES_OSIMMOCOCOPGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: RegisterTypes_osimMocoCOPGoal.h                                   *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoCOPGoalDLL.h"

extern "C" {

OSIMMOCOCOPGOAL_API void RegisterTypes_osimMocoCOPGoal();

}

class osimMocoCOPGoalInstantiator {
public:
    osimMocoCOPGoalInstantiator();
private:
    void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIMMOCOCOPGOAL_H
