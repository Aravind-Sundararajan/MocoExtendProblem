#ifndef OPENSIM_OSIMMOCOCOPGOALDLL_H
#define OPENSIM_OSIMMOCOCOPGOALDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimMocoCOPGoalDLL.h                                *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#ifndef _WIN32
    #define OSIMMOCOCOPGOAL_API
#else
    #ifdef OSIMMOCOCOPGOAL_EXPORTS
        #define OSIMMOCOCOPGOAL_API __declspec(dllexport)
    #else
        #define OSIMMOCOCOPGOAL_API __declspec(dllimport)
    #endif
#endif

#endif // OPENSIM_OSIMMOCOCOPGOALDLL_H
