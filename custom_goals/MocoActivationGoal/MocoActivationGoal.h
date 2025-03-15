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

/** This goal minimizes the sum of muscle activations to a specified power, optionally allowing
 * custom weights for specific muscles. The integrand is:
 * \f[
 * \sum_i w_i |a_i|^p
 * \f]
 * where \f$ a_i \f$ are the activation values, \f$ w_i \f$ are the weights 
 * (default: 1.0), and \f$ p \f$ is the exponent (default: 2).
 */
class OSIMMOCOACTIVATIONGOAL_API MocoActivationGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoActivationGoal, MocoGoal);

public:
    /** Default constructor */
    MocoActivationGoal() { constructProperties(); }
    
    /** Constructor with name
     * @param name The name of the goal */
    MocoActivationGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    
    /** Constructor with name and weight
     * @param name The name of the goal
     * @param weight Weight for this goal term in the optimization */
    MocoActivationGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /** Set the target value when using this goal as an endpoint constraint
     * @param end_point_goal The target value (default: 0) */
    void setEndPointGoal(double end_point_goal) { set_end_point_goal(end_point_goal); }
    
    /** Get the endpoint constraint target value
     * @return The target value for endpoint constraint mode */
    double getEndPointGoal() const { return get_end_point_goal(); }

    /** Set the exponent applied to activation values
     * @param ex The exponent value. For ex > 1, absolute value is applied first */
    void setExponent(int ex) { set_exponent(ex); }
    
    /** Get the current exponent value
     * @return The exponent applied to activation values */
    bool getExponent() const { return get_exponent(); }

    /** Set the names of states to apply custom weights to
     * @param custom_weight_component_paths Vector of state names */
    void setCustomWeightNames(const std::vector<std::string> custom_weight_component_paths) {
        m_custom_state_names = custom_weight_component_paths;
    }

    /** Set the custom weight values corresponding to the custom state names
     * @param custom_weights Vector of weights matching the order of custom state names */
    void setCustomWeightValues(const std::vector<double> custom_weights) {
        m_custom_weights_input = custom_weights;
    }

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
    mutable std::vector<std::string> m_state_names;
    mutable std::vector<double> m_custom_weights;
    mutable std::vector<double> m_custom_weights_input;
    mutable std::vector<std::string> m_custom_state_names;
    mutable std::vector<int> m_custom_act_indices;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOACTIVATIONGOAL_H
