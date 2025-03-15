#ifndef OPENSIM_MOCOMARKERACCELERATIONGOAL_H
#define OPENSIM_MOCOMARKERACCELERATIONGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoMarkerAccelerationGoal.h                                      *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "osimMocoMarkerAccelerationGoalDLL.h"

namespace OpenSim {

/** @brief Goal that minimizes the acceleration of a model marker
 *
 * This goal minimizes the absolute acceleration of a specified model marker,
 * integrated over the phase. The acceleration is computed as the sum of the
 * squared accelerations in the x, y, and z directions.
 *
 * This goal can be useful for:
 * - Generating smoother marker trajectories
 * - Reducing rapid changes in marker motion
 * - Minimizing jerk in specific body segments
 * - Improving motion naturalness
 *
 * The goal requires specification of a marker name to track. The acceleration
 * can be raised to a specified power using the exponent property, and can
 * optionally be divided by the total displacement of the model during the phase
 * to make the cost invariant to the distance traveled.
 */
class OSIMMOCOMARKERACCELERATIONGOAL_API MocoMarkerAccelerationGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoMarkerAccelerationGoal, MocoGoal);

public:
    /** @name Constructors */
    /// @{
    /** Default constructor */
    MocoMarkerAccelerationGoal() { constructProperties(); }

    /** Constructor with name
     * @param name The name of the goal */
    MocoMarkerAccelerationGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }

    /** Constructor with name and weight
     * @param name The name of the goal
     * @param weight Weight for this goal term in the optimization */
    MocoMarkerAccelerationGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        log_cout("Constructed class");
        constructProperties();
    }
    /// @}

    /** Set the name of the marker to track
     * @param name The marker name */
    void setMarkerName(std::string name) { set_marker_name(std::move(name)); }

    /** Set the exponent for the acceleration terms
     * @param ex The exponent value */
    void setExponent(int ex) { set_exponent(ex); }
    
    /** Get the current exponent value
     * @return The exponent value */
    bool getExponent() const { return get_exponent(); }

    /** Set whether to divide by displacement
     * @param tf True to divide by displacement, false otherwise */
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    
    /** Get whether the goal is divided by displacement
     * @return True if divided by displacement, false otherwise */
    bool getDivideByDisplacement() const { return get_divide_by_displacement(); }

protected:
    /** @name Required implementations of virtual methods */
    /// @{
    /** Get the default mode for this goal */
    Mode getDefaultModeImpl() const override { return Mode::Cost; }

    /** Initialize the goal with the model */
    void initializeOnModelImpl(const Model&) const override;
    
    /** Calculate the integrand value for the cost function
     * @param input Input data for the current state
     * @param integrand Reference to store the calculated integrand value */
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    
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

    OpenSim_DECLARE_PROPERTY(marker_name, std::string,
            "The name of the marker for this goal");
    /// @}

    /** Initialize the goal's properties */
    void constructProperties();

    /** @name Internal working variables */
    /// @{
    /// Function to compute power of acceleration values
    mutable std::function<double(const double&)> m_power_function;
    /// Reference to the model marker being tracked
    mutable SimTK::ReferencePtr<const Point> m_model_marker;
    /// @}
};

} // namespace OpenSim

#endif // OPENSIM_MOCOMARKERACCELERATIONGOAL_H
