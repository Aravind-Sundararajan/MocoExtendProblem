#ifndef OPENSIM_REGISTERTYPES_OSIMMOCOACTIVATIONSQUARED_H
#define OPENSIM_REGISTERTYPES_OSIMMOCOACTIVATIONSQUAREDGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: RegisterTypes_osimMocoActivationSquaredGoal.h                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi, Aravind Sundararajan                                                      *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoActivationSquaredGoalDLL.h"

extern "C" {

OSIMMOCOACTIVATIONSQUAREDGOAL_API void RegisterTypes_osimMocoActivationSquaredGoal();

}

class osimMocoActivationSquaredGoalInstantiator {
public:
    osimMocoActivationSquaredGoalInstantiator();
private:
    void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIMMOCOACTIVATIONSQUAREDGOAL_H
