#ifndef OPENSIM_REGISTERTYPES_OSIMMOCOZMP_H
#define OPENSIM_REGISTERTYPES_OSIMMOCOMUSCLESTRAINGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: RegisterTypes_osimMocoMuscleStrainGoal.h *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoMuscleStrainGoalDLL.h"

extern "C" {

OSIMMOCOMUSCLESTRAINGOAL_API void RegisterTypes_osimMocoMuscleStrainGoal();
}

class osimMocoMuscleStrainGoalInstantiator {
public:
  osimMocoMuscleStrainGoalInstantiator();

private:
  void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIMMOCOMUSCLESTRAINGOAL_H
