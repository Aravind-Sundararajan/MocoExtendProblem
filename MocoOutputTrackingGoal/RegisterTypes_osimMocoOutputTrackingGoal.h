#ifndef OPENSIM_REGISTERTYPES_OSIMMOCOOutputTracking_H
#define OPENSIM_REGISTERTYPES_OSIMMOCOOutputTrackingGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: RegisterTypes_osimMocoOutputTrackingGoal.h                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi                                                     *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoOutputTrackingGoalDLL.h"

extern "C" {

OSIMMOCOOutputTrackingGOAL_API void RegisterTypes_osimMocoOutputTrackingGoal();

}

class osimMocoOutputTrackingGoalInstantiator {
public:
    osimMocoOutputTrackingGoalInstantiator();
private:
    void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIMMOCOOutputTrackingGOAL_H
