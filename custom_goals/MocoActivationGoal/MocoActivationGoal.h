#ifndef OPENSIM_MOCOACTIVATIONGOAL_H
#define OPENSIM_MOCOACTIVATIONGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoActivationGoal.h                                              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            * 
 * -------------------------------------------------------------------------- */

#include <OpenSim/Moco/osimMoco.h>
#include "osimMocoActivationGoalDLL.h"

namespace OpenSim {

class OSIMMOCOACTIVATIONGOAL_API MocoActivationGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoActivationGoal, MocoGoal);

public:
    // Make the constructors for this class
    MocoActivationGoal() { constructProperties();}
    MocoActivationGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoActivationGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    // Public methods to change and get the end-point goal value
    void setEndPointGoal(double end_point_goal) { set_end_point_goal(end_point_goal); }
    double getEndPointGoal() const { return get_end_point_goal(); }


    // Public member to set the exponent
    void setExponent(int ex) { set_exponent(ex); }
    bool getExponent() const { return get_exponent(); }

protected:
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    bool getSupportsEndpointConstraintImpl() const override { return true;}

    // Initialization function
    void initializeOnModelImpl(const Model&) const override;
    
    // Integration functions
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;

 private:
    OpenSim_DECLARE_PROPERTY(exponent, int,
        "The exponent applied to the output value in the integrand. "
        "The output can take on negative values in the integrand when the "
        "exponent is set to 1 (the default value). When the exponent is "
        "set to a value greater than 1, the absolute value function is "
        "applied to the output (before the exponent is applied), meaning "
        "that odd numbered exponents (greater than 1) do not take on "
        "negative values.");
    // Make the end_point_goal property
    OpenSim_DECLARE_PROPERTY(end_point_goal, double,
        "Target value for end-point goal (default: 0)");

    // Make a function to set the default values of these properties
    void constructProperties();

    // Make a private member that stores all the indices for muscle activations
    mutable std::vector<int> m_act_indices;
    mutable std::function<double(const double &)> m_power_function;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOACTIVATIONGOAL_H
