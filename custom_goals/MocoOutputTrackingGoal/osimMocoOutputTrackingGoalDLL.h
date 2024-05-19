#ifndef OPENSIM_OSIMMOCOOUTPUTTRACKINGGOALDLL_H
#define OPENSIM_OSIMMOCOOUTPUTTRACKINGGOALDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimMocoOutputTrackingGoalDLL.h                                   *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#ifndef _WIN32
#define OSIMMOCOOUTPUTTRACKINGGOAL_API
#else
#ifdef OSIMMOCOOUTPUTTRACKINGGOAL_EXPORTS
#define OSIMMOCOOUTPUTTRACKINGGOAL_API __declspec(dllexport)
#else
#define OSIMMOCOOUTPUTTRACKINGGOAL_API __declspec(dllimport)
#endif
#endif

#endif // OPENSIM_OSIMMOCOOUTPUTTRACKINGGOALDLL_H
