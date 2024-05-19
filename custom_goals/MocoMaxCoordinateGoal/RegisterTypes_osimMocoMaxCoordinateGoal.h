#ifndef OPENSIM_REGISTERTYPES_OSIMMOCOMAXCOORDINATE_H
#define OPENSIM_REGISTERTYPES_OSIMMOCOMAXCOORDINATEGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: RegisterTypes_osimMocoMaxCoordinateGoal.h                         *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                               *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoMaxCoordinateGoalDLL.h"

extern "C" {

OSIMMOCOMAXCOORDINATEGOAL_API void RegisterTypes_osimMocoMaxCoordinateGoal();

}

class osimMocoMaxCoordinateGoalInstantiator {
public:
    osimMocoMaxCoordinateGoalInstantiator();
private:
    void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIMMOCOMAXCOORDINATEGOAL_H
