#ifndef OPENSIM_MOCOMARKERACCELERATIONGOAL_H
#define OPENSIM_MOCOMARKERACCELERATIONGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoMarkerAccelerationGoal.h                                      *
 * -------------------------------------------------------------------------- *
  *                                                                           *
 * Author(s): Varun Joshi, Aravind Sundararajan                                                      *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "osimMocoMarkerAccelerationGoalDLL.h"

namespace OpenSim {

/** 
 * @brief Goal that minimizes marker acceleration in a Moco optimization problem
 *
 * This goal minimizes the absolute acceleration of a model marker summed over its
 * x, y, and z components, integrated over the phase. The integrand is:
 * \f[
 * |\ddot{p}|^p = (|\ddot{p}_x| + |\ddot{p}_y| + |\ddot{p}_z|)^p
 * \f]
 * where \f$ \ddot{p} \f$ is the marker acceleration and \f$ p \f$ is the
 * exponent (default: 2).
 *
 * This goal can be useful for:
 * - Reducing rapid changes in marker motion
 * - Smoothing marker trajectories
 * - Minimizing jerk in marker paths
 *
 * @note The marker must exist in the model and its name must be specified using
 * setMarkerName() before the goal can be used.
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

    /** Set the name of the marker whose acceleration should be minimized
     * @param name Name of the marker in the model */
    void setMarkerName(std::string name) { set_marker_name(std::move(name)); }

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
    /// Reference to the marker in the model
    mutable SimTK::ReferencePtr<const Point> m_model_marker;
    /// @}
};

} // namespace OpenSim

#endif // OPENSIM_MOCOMARKERACCELERATIONGOAL_H
