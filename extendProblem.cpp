#include <Simbody.h>
#include <OpenSim/OpenSim.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include "mexplus.h"
#include <vector>
#include <string>
#include "C:\Users\oneill_lab\Desktop\MocoExtendProblem/custom_goals/MocoActivationSquaredGoal/MocoActivationSquaredGoal.h"
#include "C:\Users\oneill_lab\Desktop\MocoExtendProblem/custom_goals/MocoCoordinateAccelerationGoal/MocoCoordinateAccelerationGoal.h"
#include "C:\Users\oneill_lab\Desktop\MocoExtendProblem/custom_goals/MocoMarkerAccelerationGoal/MocoMarkerAccelerationGoal.h"
#include "C:\Users\oneill_lab\Desktop\MocoExtendProblem/custom_goals/MocoMaxCoordinateGoal/MocoMaxCoordinateGoal.h"
#include "C:\Users\oneill_lab\Desktop\MocoExtendProblem/custom_goals/MocoOutputTrackingGoal/MocoOutputTrackingGoal.h"
#include "C:\Users\oneill_lab\Desktop\MocoExtendProblem/custom_goals/MocoZMPGoal/MocoZMPGoal.h"
using namespace std;
using namespace mexplus;
using namespace OpenSim;
using namespace SimTK;


class extendProblem
{
public:
    extendProblem(const __int64 p){
        m_p = (OpenSim::MocoProblem *) p;
        mexPrintf("hello MocoProblem!\n");
    }
    ~extendProblem(){
        mexPrintf("goodbye MocoProblem!\n");
    }
    void addMocoActivationSquaredGoal(const string& goalName, double weight, bool tf, double end_point_goal){
        mexPrintf("Adding MocoActivationSquaredGoal goal\n");
        auto* goal = m_p->addGoal<MocoActivationSquaredGoal>(goalName, weight);
        goal->setDivideByDisplacement(tf);
        goal->setEndPointGoal(end_point_goal);
    }
    void addMocoCoordinateAccelerationGoal(const string& goalName, double weight, bool tf, std::vector<std::string> refCoordNames){
        mexPrintf("Adding MocoCoordinateAccelerationGoal goal\n");
        auto* goal = m_p->addGoal<MocoCoordinateAccelerationGoal>(goalName, weight);
        goal->setDivideByDisplacement(tf);
        goal->setStateNames(refCoordNames);
    }
    void addMocoMarkerAccelerationGoal(const string& goalName, double weight, std::string name, bool tf){
        mexPrintf("Adding MocoMarkerAccelerationGoal goal\n");
        auto* goal = m_p->addGoal<MocoMarkerAccelerationGoal>(goalName, weight);
        goal->setMarkerName(name);
        goal->setDivideByDisplacement(tf);
    }
    void addMocoMaxCoordinateGoal(const string& goalName, double weight, bool tf){
        mexPrintf("Adding MocoMaxCoordinateGoal goal\n");
        auto* goal = m_p->addGoal<MocoMaxCoordinateGoal>(goalName, weight);
        goal->setDivideByDisplacement(tf);
    }
    void addMocoOutputTrackingGoal(const string& goalName, double weight, bool tf, double end_point_goal){
        mexPrintf("Adding MocoOutputTrackingGoal goal\n");
        auto* goal = m_p->addGoal<MocoOutputTrackingGoal>(goalName, weight);
        goal->setDivideByDisplacement(tf);
        goal->setEndPointGoal(end_point_goal);
    }
    void addMocoZMPGoal(const string& goalName, double weight, bool tf){
        mexPrintf("Adding MocoZMPGoal goal\n");
        auto* goal = m_p->addGoal<MocoZMPGoal>(goalName, weight);
        goal->setDivideByDisplacement(tf);
    }

private:
    OpenSim::MocoProblem *m_p = NULL;
};




template class mexplus::Session<extendProblem>;

namespace
{
    MEX_DEFINE(new) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 1);
        OutputArguments output(nlhs, plhs, 1);
        output.set(0, Session<extendProblem>::create(new extendProblem(input.get<__int64>(0))));
    }
    MEX_DEFINE(delete) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 1);
        OutputArguments output(nlhs, plhs, 0);
        Session<extendProblem>::destroy(input.get(0));
    }

    MEX_DEFINE(addMocoActivationSquaredGoal) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 5);
        OutputArguments output(nlhs, plhs, 0);
        extendProblem* engine = Session<extendProblem>::get(input.get(0));
        engine->addMocoActivationSquaredGoal(input.get<string>(1),input.get<double>(2),input.get<bool>(3),input.get<double>(4));
    }
    MEX_DEFINE(addMocoCoordinateAccelerationGoal) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 5);
        OutputArguments output(nlhs, plhs, 0);
        extendProblem* engine = Session<extendProblem>::get(input.get(0));
        engine->addMocoCoordinateAccelerationGoal(input.get<string>(1),input.get<double>(2),input.get<bool>(3),input.get<std::vector<std::string>>(4));
    }
    MEX_DEFINE(addMocoMarkerAccelerationGoal) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 5);
        OutputArguments output(nlhs, plhs, 0);
        extendProblem* engine = Session<extendProblem>::get(input.get(0));
        engine->addMocoMarkerAccelerationGoal(input.get<string>(1),input.get<double>(2),input.get<std::string>(3),input.get<bool>(4));
    }
    MEX_DEFINE(addMocoMaxCoordinateGoal) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 4);
        OutputArguments output(nlhs, plhs, 0);
        extendProblem* engine = Session<extendProblem>::get(input.get(0));
        engine->addMocoMaxCoordinateGoal(input.get<string>(1),input.get<double>(2),input.get<bool>(3));
    }
    MEX_DEFINE(addMocoOutputTrackingGoal) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 5);
        OutputArguments output(nlhs, plhs, 0);
        extendProblem* engine = Session<extendProblem>::get(input.get(0));
        engine->addMocoOutputTrackingGoal(input.get<string>(1),input.get<double>(2),input.get<bool>(3),input.get<double>(4));
    }
    MEX_DEFINE(addMocoZMPGoal) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 4);
        OutputArguments output(nlhs, plhs, 0);
        extendProblem* engine = Session<extendProblem>::get(input.get(0));
        engine->addMocoZMPGoal(input.get<string>(1),input.get<double>(2),input.get<bool>(3));
    }


} //namespace

MEX_DISPATCH


