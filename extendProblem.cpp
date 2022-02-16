//to build:
//make sure mex is building with VS
//mex -I"C:\libs" -I"C:\Users\asundar4\Desktop\MocoCustomGoal\MocoZMPGoal" -I"C:\opensim_install\sdk\spdlog\include" -I"C:\opensim_install\sdk\Simbody\include" -I"C:\opensim_install\sdk\include" -I"C:\opensim_install\sdk\include\OpenSim" -L"C:\opensim_install\sdk\Simbody\lib" -L"C:\Users\asundar4\Desktop\MocoCustomGoal\build\RelWithDebInfo" -lSimTKcommon -lSimTKsimbody -lSimTKmath -L"C:\opensim_install\sdk\lib" -losimActuators -losimExampleComponents -losimSimulation -losimAnalyses -losimJavaJNI -losimTools -losimMoco -losimMocoZMPGoal -losimCommon -losimLepton -losimTools extendProblem.cpp
#include <Simbody.h>
#include <OpenSim/OpenSim.h>
#include <OpenSim/Moco/osimMoco.h>
#include "MocoZMPGoal.h"
#include "mexplus.h"
#include <vector>
#include <string>

using namespace std;
using namespace mexplus;
using namespace OpenSim;
using namespace SimTK;


class extendProblem
{
public:
    extendProblem(const __int64 p){
        m_p = (OpenSim::MocoProblem *) p;
    }
    ~extendProblem(){
        //free(m_p); //we don't have to free since MATLAB is instantiating the problem through the API
    }
    
    void addCustomGoal(double weight)
    {
        mexPrintf("ADDING A CUSTOM GOAL\n");
        auto* goal = m_p->addGoal<MocoZMPGoal>("ZMP", weight);
        goal->setDivideByDisplacement(true);
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
    MEX_DEFINE(addCustomGoal) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
    {
        InputArguments input(nrhs, prhs, 2);
        OutputArguments output(nlhs, plhs, 0);
        extendProblem* engine = Session<extendProblem>::get(input.get(0));
        engine->addCustomGoal(input.get<double>(1));
    }
    
} //namespace

MEX_DISPATCH