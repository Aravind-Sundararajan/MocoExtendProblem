#ifndef OPENSIM_OSIMMOCOZMPGOALDLL_H
#define OPENSIM_OSIMMOCOZMPGOALDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimMocoZMPGoalDLL.h                                *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 * -------------------------------------------------------------------------- */

#ifndef _WIN32
    #define OSIMMOCOZMPGOAL_API
#else
    #ifdef OSIMMOCOZMPGOAL_EXPORTS
        #define OSIMMOCOZMPGOAL_API __declspec(dllexport)
    #else
        #define OSIMMOCOZMPGOAL_API __declspec(dllimport)
    #endif
#endif

#endif // OPENSIM_OSIMMOCOZMPGOALDLL_H
