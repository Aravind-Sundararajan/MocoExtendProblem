#ifndef OPENSIM_OSIMMOCOACTIVATIONGOALDLL_H
#define OPENSIM_OSIMMOCOACTIVATIONGOALDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimMocoActivationGoalDLL.h                                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi                                                     *
 * -------------------------------------------------------------------------- */

#ifndef _WIN32
    #define OSIMMOCOACTIVATIONGOAL_API
#else
    #ifdef OSIMMOCOACTIVATIONGOAL_EXPORTS
        #define OSIMMOCOACTIVATIONGOAL_API __declspec(dllexport)
    #else
        #define OSIMMOCOACTIVATIONGOAL_API __declspec(dllimport)
    #endif
#endif

#endif // OPENSIM_OSIMMOCOACTIVATIONGOALDLL_H
