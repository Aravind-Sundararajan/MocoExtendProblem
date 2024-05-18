#ifndef OPENSIM_REGISTERTYPES_OSIMMOCOMARKERACCELERATIONGOAL_H
#define OPENSIM_REGISTERTYPES_OSIMMOCOMARKERACCELERATIONGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: RegisterTypes_osimMocoMarkerAccelerationGoal.h                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi, Aravind Sundararajan                                                      *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoMarkerAccelerationGoalDLL.h"

extern "C" {

OSIMMOCOMARKERACCELERATIONGOAL_API void RegisterTypes_osimMocoMarkerAccelerationGoal();

}

class osimMocoMarkerAccelerationGoalInstantiator {
public:
    osimMocoMarkerAccelerationGoalInstantiator();
private:
    void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIMMOCOMARKERACCELERATIONGOAL_H
