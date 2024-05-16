#ifndef OPENSIM_REGISTERTYPES_OSIMMOCOCOORDINATEACCELERATION_H
#define OPENSIM_REGISTERTYPES_OSIMMOCOCOORDINATEACCELERATIONGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: RegisterTypes_osimMocoCoordinateAccelerationGoal.h                *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoCoordinateAccelerationGoalDLL.h"

extern "C" {

OSIMMOCOCOORDINATEACCELERATIONGOAL_API void RegisterTypes_osimMocoCoordinateAccelerationGoal();

}

class osimMocoCoordinateAccelerationGoalInstantiator {
public:
    osimMocoCoordinateAccelerationGoalInstantiator();
private:
    void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIMMOCOCOORDINATEACCELERATIONGOAL_H
