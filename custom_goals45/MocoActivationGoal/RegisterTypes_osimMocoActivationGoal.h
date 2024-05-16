#ifndef OPENSIM_REGISTERTYPES_OSIMMOCOACTIVATION_H
#define OPENSIM_REGISTERTYPES_OSIMMOCOACTIVATIONGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: RegisterTypes_osimMocoActivationGoal.h                            *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoActivationGoalDLL.h"

extern "C" {

OSIMMOCOACTIVATIONGOAL_API void RegisterTypes_osimMocoActivationGoal();

}

class osimMocoActivationGoalInstantiator {
public:
    osimMocoActivationGoalInstantiator();
private:
    void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIMMOCOACTIVATIONGOAL_H
