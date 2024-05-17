#ifndef OPENSIM_REGISTERTYPES_OSIMMOCOBOS_H
#define OPENSIM_REGISTERTYPES_OSIMMOCOBOSGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: RegisterTypes_osimMocoBOSGoal.h                                   *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoBOSGoalDLL.h"

extern "C" {

OSIMMOCOBOSGOAL_API void RegisterTypes_osimMocoBOSGoal();

}

class osimMocoBOSGoalInstantiator {
public:
    osimMocoBOSGoalInstantiator();
private:
    void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIMMOCOBOSGOAL_H
