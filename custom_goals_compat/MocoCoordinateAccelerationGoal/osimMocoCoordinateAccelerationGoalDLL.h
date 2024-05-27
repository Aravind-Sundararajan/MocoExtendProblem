#ifndef OPENSIM_OSIMMOCOCOORDINATEACCELERATIONGOALDLL_H
#define OPENSIM_OSIMMOCOCOORDINATEACCELERATIONGOALDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimMocoCoordinateAccelerationGoalDLL.h                                *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#ifndef _WIN32
    #define OSIMMOCOCOORDINATEACCELERATIONGOAL_API
#else
    #ifdef OSIMMOCOCOORDINATEACCELERATIONGOAL_EXPORTS
        #define OSIMMOCOCOORDINATEACCELERATIONGOAL_API __declspec(dllexport)
    #else
        #define OSIMMOCOCOORDINATEACCELERATIONGOAL_API __declspec(dllimport)
    #endif
#endif

#endif // OPENSIM_OSIMMOCOCOORDINATEACCELERATIONGOALDLL_H
