#ifndef OPENSIM_MOCOZMPGOAL_H
#define OPENSIM_MOCOZMPGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoZMPGoal.h                                                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */


#include <OpenSim/Moco/osimMoco.h>
#include "osimMocoZMPGoalDLL.h"

namespace OpenSim {

/** @brief Goal that minimizes deviation of the Zero Moment Point (ZMP) from a desired location
 *
 * This goal minimizes the distance between the actual Zero Moment Point (ZMP) and
 * a target location. The ZMP is the point where the horizontal component of the
 * moment of ground reaction forces equals zero. The integrand is:
 * \f[
 * |p_{zmp} - p_{target}|^p
 * \f]
 * where \f$ p_{zmp} \f$ is the current ZMP location, \f$ p_{target} \f$ is the
 * target location, and \f$ p \f$ is the exponent (default: 2).
 *
 * This goal is useful for:
 * - Maintaining dynamic balance during motion
 * - Controlling the center of pressure location
 * - Generating stable walking motions
 *
 * The ZMP is computed using ground reaction forces and moments from contact
 * forces in the model.
 */
class OSIMMOCOZMPGOAL_API MocoZMPGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoZMPGoal, MocoGoal);

public:
    /** @name Constructors */
    /// @{
    /** Default constructor */
    MocoZMPGoal() { constructProperties(); }

    /** Constructor with name
     * @param name The name of the goal */
    MocoZMPGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }

    /** Constructor with name and weight
     * @param name The name of the goal
     * @param weight Weight for this goal term in the optimization */
    MocoZMPGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }
    /// @}

    /** Set the exponent for the goal
     * @param ex The exponent value. For ex > 1, absolute value is applied first */
    void setExponent(int ex) { set_exponent(ex); }
    
    /** Get the current exponent value
     * @return The exponent applied to the ZMP deviation */
    bool getExponent() const { return get_exponent(); }

protected:
    /** @name Required implementations of virtual methods */
    /// @{
    /** Get the default mode for this goal */
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    
    /** Whether this goal supports endpoint constraint mode */
    bool getSupportsEndpointConstraintImpl() const override { return false; }
    
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
    OpenSim_DECLARE_PROPERTY(exponent, int,
            "The exponent applied to the output value in the integrand. "
            "The output can take on negative values in the integrand when the "
            "exponent is set to 1 (the default value). When the exponent is "
            "set to a value greater than 1, the absolute value function is "
            "applied to the output (before the exponent is applied), meaning "
            "that odd numbered exponents (greater than 1) do not take on "
            "negative values.");
    /// @}

    /** Initialize the goal's properties */
    void constructProperties();

    /** @name Internal working variables */
    /// @{
    /// Names of forces to consider for ZMP calculation
    mutable std::vector<std::string> m_force_names;
    /// Function to compute power of ZMP deviation
    mutable std::function<double(const double&)> m_power_function;
    /// Reference to the model
    mutable OpenSim::Model m_model;
    /// @}
};

} // namespace OpenSim

#endif // OPENSIM_MOCOZMPGOAL_H
