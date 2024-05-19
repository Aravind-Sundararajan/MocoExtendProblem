#ifndef OPENSIM_OSIMMOCOMARKERACCELERATIONGOALDLL_H
#define OPENSIM_OSIMMOCOMARKERACCELERATIONGOALDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimMocoMarkerAccelerationGoalDLL.h                               *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi, Aravind Sundararajan                                                      *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#ifndef _WIN32
    #define OSIMMOCOMARKERACCELERATIONGOAL_API
#else
    #ifdef OSIMMOCOMARKERACCELERATIONGOAL_EXPORTS
        #define OSIMMOCOMARKERACCELERATIONGOAL_API __declspec(dllexport)
    #else
        #define OSIMMOCOMARKERACCELERATIONGOAL_API __declspec(dllimport)
    #endif
#endif

#endif // OPENSIM_OSIMMOCOMARKERACCELERATIONGOALDLL_H
