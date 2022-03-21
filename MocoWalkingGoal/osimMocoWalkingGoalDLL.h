#ifndef OPENSIM_OSIMMOCOWALKINGGOALDLL_H
#define OPENSIM_OSIMMOCOWALKINGGOALDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimMocoWalkingGoalDLL.h                                *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 * -------------------------------------------------------------------------- */

#ifndef _WIN32
    #define OSIMMOCOWALKINGGOAL_API
#else
    #ifdef OSIMMOCOWALKINGGOAL_EXPORTS
        #define OSIMMOCOWALKINGGOAL_API __declspec(dllexport)
    #else
        #define OSIMMOCOWALKINGGOAL_API __declspec(dllimport)
    #endif
#endif

#endif // OPENSIM_OSIMMOCOWALKINGGOALDLL_H
