/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoMaxCoordinateGoal.cpp                                              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 * this uses the LSE                                                          *
 * (a smooth approx. of the maximum function that is C2 continuous)           *
 * -------------------------------------------------------------------------- */

#include "MocoMaxCoordinateGoal.h"

using namespace OpenSim;

void MocoMaxCoordinateGoal::constructProperties() {
    constructProperty_divide_by_displacement(false);
    StatesTrajectory st = StatesTrajectory();
    std::vector<double> inte;
}

void MocoMaxCoordinateGoal::initializeOnModelImpl(const Model& model) const {
    setRequirements(1, 1);
}

void MocoMaxCoordinateGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    const auto& state = input.state;
    getModel().realizeDynamics(input.state);
    inte.push_back(getWeight()*double(state.getU().get(0)));
    integrand = state.getU().get(0);
    //st.append(state);
}

void MocoMaxCoordinateGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    double integral = 0;
    double se = 0;
    double lse = 0;
    
    //for (auto& state : st){
    //    inte.push_back(double(state.getQ().get(0)));
    //}
    double m = *max_element(std::begin(inte), std::end(inte));
    for (int i =0; i < inte.size(); i++){
        se += exp(inte[i] - m);
    }
    lse = log(se) + m;
    cost[0] = lse;
}