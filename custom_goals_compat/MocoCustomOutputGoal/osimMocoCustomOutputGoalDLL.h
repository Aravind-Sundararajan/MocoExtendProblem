#ifndef OPENSIM_OSIMMOCOCUSTOMOUTPUTGOALDLL_H
#define OPENSIM_OSIMMOCOCUSTOMOUTPUTGOALDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimMocoCustomOutputGoalDLL.h                                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                               *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#ifndef _WIN32
    #define OSIMMOCOCUSTOMOUTPUTGOAL_API
#else
    #ifdef OSIMMOCOCUSTOMOUTPUTGOAL_EXPORTS
        #define OSIMMOCOCUSTOMOUTPUTGOAL_API __declspec(dllexport)
    #else
        #define OSIMMOCOCUSTOMOUTPUTGOAL_API __declspec(dllimport)
    #endif
#endif

#endif // OPENSIM_OSIMMOCOCUSTOMOUTPUTGOALDLL_H
