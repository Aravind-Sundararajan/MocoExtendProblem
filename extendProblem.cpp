//to build:
//make sure mex is building with VS
//mex -I"C:\libs" -I"C:\Users\oneill_lab\Desktop\MOCOPROJECTS\MocoExtendProblem\MocoCoordinateAccelerationGoal"  -I"C:\Users\oneill_lab\Desktop\MOCOPROJECTS\MocoExtendProblem\MocoMaxCoordinateGoal" -I"C:\Users\oneill_lab\Desktop\MOCOPROJECTS\MocoExtendProblem\MocoActivationSquaredGoal" -I"C:\Users\oneill_lab\Desktop\MOCOPROJECTS\MocoExtendProblem\MocoZMPGoal" -I"C:\opensim_install\sdk\spdlog\include" -I"C:\opensim_install\sdk\Simbody\include" -I"C:\opensim_install\sdk\include" -I"C:\opensim_install\sdk\include\OpenSim" -L"C:\opensim_install\sdk\Simbody\lib" -L"C:\Users\oneill_lab\Desktop\MOCOPROJECTS\MocoExtendProblem\build\RelWithDebInfo" -lSimTKcommon -lSimTKsimbody -lSimTKmath -L"C:\opensim_install\sdk\lib" -losimActuators -losimExampleComponents -losimSimulation -losimAnalyses -losimJavaJNI -losimTools -losimMocoActivationSquaredGoal -losimMocoZMPGoal -losimMocoCoordinateAccelerationGoal -losimMocoMaxCoordinateGoal -losimMoco -losimCommon -losimLepton -losimTools extendProblem.cpp
// 
#include <Simbody.h>
#include <OpenSim/OpenSim.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include "mexplus.h"
#include <vector>
#include <string>

#include "MocoCoordinateAccelerationGoal.h"
#include "MocoMaxCoordinateGoal.h"
#include "MocoMarkerAccelerationGoal.h"
#include "MocoActivationSquaredGoal.h"
#include "MocoZMPGoal.h"


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
    
    void addAccelerationGoal(double weight, vector<string> coordNames, bool displacementDiv)
    {
        mexPrintf("ADDING COORDINATE ACCELERATIONS GOAL\n");
        auto* goal = m_p->addGoal<MocoCoordinateAccelerationGoal>("CoordinateAcceleration", weight);
        mexPrintf("ADDING COORDINATE ACCELERATIONS GOAL\n");
        goal->setDivideByDisplacement(displacementDiv);
        mexPrintf("Setting COORDINATE Names \n");
        goal->setStateNames(coordNames);
    }
    void addActivationGoal(double weight)
    {
        mexPrintf("ADDING ACTIVATION SQUARED GOAL\n");
        auto* goal = m_p->addGoal<MocoActivationSquaredGoal>("Activations", weight);
        //goal->setDivideByDisplacement(true);
    }
    void addZMPGoal(double weight)
    {
        mexPrintf("ADDING ZMP GOAL\n");
        auto* goal = m_p->addGoal<MocoZMPGoal>("ZMP", weight);
        //goal->setDivideByDisplacement(true);
    }
    void addMaxCoordinateGoal(double weight)
    {
        mexPrintf("ADDING MAX COORDINATE GOAL\n");
        auto* goal = m_p->addGoal<MocoMaxCoordinateGoal>("maximize", weight);
        //goal->setDivideByDisplacement(true);
    }
    void addMarkerGoal(double weight, const string& markerName, bool displacementDiv)
    {
        mexPrintf("ADDING Marker GOAL\n");
        auto* goal = m_p->addGoal<MocoMarkerAccelerationGoal>("Marker", weight);
        goal->setDivideByDisplacement(displacementDiv);
        mexPrintf("Setting Marker Name \n");
        goal->setMarkerName(markerName);
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
    MEX_DEFINE(addAccelerationGoal) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 4);
        OutputArguments output(nlhs, plhs, 0);
        extendProblem* engine = Session<extendProblem>::get(input.get(0));
        engine->addAccelerationGoal(input.get<double>(1), input.get<vector<string>>(2), input.get<bool>(3));
    }
    MEX_DEFINE(addActivationGoal) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 2);
        OutputArguments output(nlhs, plhs, 0);
        extendProblem* engine = Session<extendProblem>::get(input.get(0));
        engine->addActivationGoal(input.get<double>(1));
    }
    MEX_DEFINE(addZMPGoal) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 2);
        OutputArguments output(nlhs, plhs, 0);
        extendProblem* engine = Session<extendProblem>::get(input.get(0));
        engine->addZMPGoal(input.get<double>(1));
    }
    MEX_DEFINE(addMaxCoordinateGoal) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 2);
        OutputArguments output(nlhs, plhs, 0);
        extendProblem* engine = Session<extendProblem>::get(input.get(0));
        engine->addMaxCoordinateGoal(input.get<double>(1));
    }
    MEX_DEFINE(addMarkerGoal) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 4);
        OutputArguments output(nlhs, plhs, 0);
        extendProblem* engine = Session<extendProblem>::get(input.get(0));
        engine->addMarkerGoal(input.get<double>(1), input.get<string>(2), input.get<bool>(3));
    }
    
} //namespace

MEX_DISPATCH