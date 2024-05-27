#ifndef OPENSIM_OSIMMOCOMAXCOORDINATEGOALDLL_H
#define OPENSIM_OSIMMOCOMAXCOORDINATEGOALDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimMocoMaxCoordinateGoalDLL.h                                *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#ifndef _WIN32
    #define OSIMMOCOMAXCOORDINATEGOAL_API
#else
    #ifdef OSIMMOCOMAXCOORDINATEGOAL_EXPORTS
        #define OSIMMOCOMAXCOORDINATEGOAL_API __declspec(dllexport)
    #else
        #define OSIMMOCOMAXCOORDINATEGOAL_API __declspec(dllimport)
    #endif
#endif

#endif // OPENSIM_OSIMMOCOMAXCOORDINATEGOALDLL_H
