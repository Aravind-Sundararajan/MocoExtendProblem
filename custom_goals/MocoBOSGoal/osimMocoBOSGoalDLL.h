#ifndef OPENSIM_OSIMMOCOBOSGOALDLL_H
#define OPENSIM_OSIMMOCOBOSGOALDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimMocoBOSGoalDLL.h                                              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#ifndef _WIN32
    #define OSIMMOCOBOSGOAL_API
#else
    #ifdef OSIMMOCOBOSGOAL_EXPORTS
        #define OSIMMOCOBOSGOAL_API __declspec(dllexport)
    #else
        #define OSIMMOCOBOSGOAL_API __declspec(dllimport)
    #endif
#endif

#endif // OPENSIM_OSIMMOCOBOSGOALDLL_H
