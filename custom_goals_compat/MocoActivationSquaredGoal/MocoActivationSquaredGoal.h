#ifndef OPENSIM_MOCOACTIVATIONSQUAREDGOAL_H
#define OPENSIM_MOCOACTIVATIONSQUAREDGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoActivationSquaredGoal.h                                       *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Moco/osimMoco.h>
#include "osimMocoActivationSquaredGoalDLL.h"

namespace OpenSim {

/** @brief Goal that minimizes the squared sum of muscle activations
 *
 * This goal minimizes the sum of squared muscle activation values,
 * integrated over the phase. The integrand is:
 * \f[
 * \sum_i a_i^2
 * \f]
 * where \f$ a_i \f$ are the activation values. This is equivalent to 
 * MocoActivationGoal with exponent=2, but may be more computationally efficient.
 *
 * This goal can be useful for:
 * - Minimizing overall muscle effort
 * - Finding metabolically efficient motions
 * - Reducing co-contraction
 *
 * The goal can optionally be divided by the total displacement of the model
 * during the phase to make the cost invariant to the distance traveled.
 */
class OSIMMOCOACTIVATIONSQUAREDGOAL_API MocoActivationSquaredGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoActivationSquaredGoal, MocoGoal);

public:
    /** @name Constructors */
    /// @{
    /** Default constructor */
    MocoActivationSquaredGoal() { constructProperties(); }

    /** Constructor with name
     * @param name The name of the goal */
    MocoActivationSquaredGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }

    /** Constructor with name and weight
     * @param name The name of the goal
     * @param weight Weight for this goal term in the optimization */
    MocoActivationSquaredGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }
    /// @}

    /** Set whether to divide the sum of squared activations by displacement
     * @param tf True to divide by displacement, false otherwise */
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    
    /** Get whether the sum is divided by displacement
     * @return True if divided by displacement, false otherwise */
    bool getDivideByDisplacement() const { return get_divide_by_displacement(); }

    /** Set the target value when using this goal as an endpoint constraint
     * @param end_point_goal The target value (default: 0) */
    void setEndPointGoal(double end_point_goal) { set_end_point_goal(end_point_goal); }
    
    /** Get the endpoint constraint target value
     * @return The target value for endpoint constraint mode */
    double getEndPointGoal() const { return get_end_point_goal(); }

protected:
    /** @name Required implementations of virtual methods */
    /// @{
    /** Get the default mode for this goal */
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    
    /** Whether this goal supports endpoint constraint mode */
    bool getSupportsEndpointConstraintImpl() const override { return true; }

    /** Initialize the goal with the model */
    void initializeOnModelImpl(const Model&) const override;
    
    /** Calculate the integrand value for the cost function
     * @param input Input data for the current state
     * @param integrand Reference to store the calculated integrand value */
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    
    /** Calculate the goal value 
     * @param input Input data containing the integral
     * @param cost Vector to store the calculated cost */
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    /// @}

private:
    /** @name Properties */
    /// @{
    OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
        "Divide by the model's displacement over the phase (default: "
        "false)");

    OpenSim_DECLARE_PROPERTY(exponent, int,
        "The exponent applied to the output value in the integrand. "
        "The output can take on negative values in the integrand when the "
        "exponent is set to 1 (the default value). When the exponent is "
        "set to a value greater than 1, the absolute value function is "
        "applied to the output (before the exponent is applied), meaning "
        "that odd numbered exponents (greater than 1) do not take on "
        "negative values.");

    OpenSim_DECLARE_PROPERTY(end_point_goal, double,
        "Target value for end-point goal (default: 0)");
    /// @}

    /** Initialize the goal's properties */
    void constructProperties();

    /** @name Internal working variables */
    /// @{
    /// Function to compute power of activation values
    mutable std::function<double(const double&)> m_power_function;
    /// Indices of activation states in the model
    mutable std::vector<int> m_act_indices;
    /// @}
};

} // namespace OpenSim

#endif // OPENSIM_MOCOACTIVATIONSQUAREDGOAL_H
