#ifndef OPENSIM_REGISTERTYPES_OSIMMOCOZMP_H
#define OPENSIM_REGISTERTYPES_OSIMMOCOZMPGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: RegisterTypes_osimMocoZMPGoal.h                                   *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoZMPGoalDLL.h"

extern "C" {

OSIMMOCOZMPGOAL_API void RegisterTypes_osimMocoZMPGoal();
}

class osimMocoZMPGoalInstantiator {
public:
  osimMocoZMPGoalInstantiator();

private:
  void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIMMOCOZMPGOAL_H
