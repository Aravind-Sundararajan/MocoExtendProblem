#ifndef OPENSIM_OSIMMOCOACTIVATIONSQUAREDGOALDLL_H
#define OPENSIM_OSIMMOCOACTIVATIONSQUAREDGOALDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimMocoActivationSquaredGoalDLL.h                                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi                                                     *
 * -------------------------------------------------------------------------- */

#ifndef _WIN32
    #define OSIMMOCOACTIVATIONSQUAREDGOAL_API
#else
    #ifdef OSIMMOCOACTIVATIONSQUAREDGOAL_EXPORTS
        #define OSIMMOCOACTIVATIONSQUAREDGOAL_API __declspec(dllexport)
    #else
        #define OSIMMOCOACTIVATIONSQUAREDGOAL_API __declspec(dllimport)
    #endif
#endif

#endif // OPENSIM_OSIMMOCOACTIVATIONSQUAREDGOALDLL_H
