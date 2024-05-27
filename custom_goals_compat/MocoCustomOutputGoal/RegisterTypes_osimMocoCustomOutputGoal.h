#ifndef OPENSIM_REGISTERTYPES_OSIMMOCOCUSTOMOUTPUT_H
#define OPENSIM_REGISTERTYPES_OSIMMOCOCUSTOMOUTPUTGOAL_H
#include "osimMocoCustomOutputGoalDLL.h"

extern "C" {

OSIMMOCOCUSTOMOUTPUTGOAL_API void RegisterTypes_osimMocoCustomOutputGoal();

}

class osimMocoCustomOutputGoalInstantiator {
public:
    osimMocoCustomOutputGoalInstantiator();
private:
    void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIMMOCOCUSTOMOUTPUTGOAL_H
