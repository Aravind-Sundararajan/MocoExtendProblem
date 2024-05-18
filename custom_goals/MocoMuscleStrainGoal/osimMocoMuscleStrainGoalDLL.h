#ifndef OPENSIM_OSIMMOCOMUSCLESTRAINGOALDLL_H
#define OPENSIM_OSIMMOCOMUSCLESTRAINGOALDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimMocoMuscleStrainGoalDLL.h                                *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#ifndef _WIN32
#define OSIMMOCOMUSCLESTRAINGOAL_API
#else
#ifdef OSIMMOCOMUSCLESTRAINGOAL_EXPORTS
#define OSIMMOCOMUSCLESTRAINGOAL_API __declspec(dllexport)
#else
#define OSIMMOCOMUSCLESTRAINGOAL_API __declspec(dllimport)
#endif
#endif

#endif // OPENSIM_OSIMMOCOMUSCLESTRAINGOALDLL_H
