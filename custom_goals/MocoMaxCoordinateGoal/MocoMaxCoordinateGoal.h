#ifndef OPENSIM_MOCOMAXCOORDINATEGOAL_H
#define OPENSIM_MOCOMAXCOORDINATEGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoMaxCoordinateGoal.h                                           *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Moco/osimMoco.h>
#include "osimMocoMaxCoordinateGoalDLL.h"

namespace OpenSim
{

/** @brief Goal that finds the maximum value of a coordinate over the motion
 *
 * This goal computes the maximum value that a specified coordinate achieves
 * during the motion. The goal value is:
 * \f[
 * \max_{t \in [t_i, t_f]} |q(t)|^p
 * \f]
 * where \f$ q(t) \f$ is the coordinate value at time t, \f$ t_i \f$ and 
 * \f$ t_f \f$ are the initial and final times, and \f$ p \f$ is the exponent 
 * (default: 2).
 *
 * This goal can be useful for:
 * - Finding peak values of joint angles
 * - Identifying maximum deviations from a neutral position
 * - Constraining the range of motion
 *
 * @note The coordinate must exist in the model and its name must be specified
 * using setStateName() before the goal can be used.
 */
class OSIMMOCOMAXCOORDINATEGOAL_API MocoMaxCoordinateGoal : public MocoGoal
{
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoMaxCoordinateGoal, MocoGoal);

public:
    /** @name Constructors */
    /// @{
    /** Default constructor */
    MocoMaxCoordinateGoal() { constructProperties(); }

    /** Constructor with name
     * @param name The name of the goal */
    MocoMaxCoordinateGoal(std::string name) : MocoGoal(std::move(name))
    {
        constructProperties();
    }

    /** Constructor with name and weight
     * @param name The name of the goal
     * @param weight Weight for this goal term in the optimization */
    MocoMaxCoordinateGoal(std::string name, double weight)
        : MocoGoal(std::move(name), weight)
    {
        constructProperties();
    }
    /// @}

    /** Set the name of the coordinate whose maximum value should be found
     * @param refCoordName Name of the coordinate in the model. Should be a state
     * variable path (e.g., '/jointset/knee_r/knee_angle_r') */
    void setStateName(const std::string refCoordName) {
        std::cout << "Setting coordinate names from string " << std::endl;
        m_state_name = refCoordName;
    }
        
protected:
    /** @name Required implementations of virtual methods */
    /// @{
    /** Get the default mode for this goal */
    Mode getDefaultModeImpl() const override { return Mode::Cost; }

    /** Whether this goal supports endpoint constraint mode */
    bool getSupportsEndpointConstraintImpl() const override { return false; }

    /** Initialize the goal with the model */
    void initializeOnModelImpl(const Model &) const override;

    /** Calculate the integrand value for the cost function
     * @param input Input data for the current state
     * @param integrand Reference to store the calculated integrand value */
    void calcIntegrandImpl(
        const IntegrandInput &input, double &integrand) const override;

    /** Calculate the goal value 
     * @param input Input data containing the integral
     * @param cost Vector to store the calculated cost */
    void calcGoalImpl(
        const GoalInput &input, SimTK::Vector &cost) const override;
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
    /// States trajectory for computing maximum
    mutable StatesTrajectory st;
    /// Vector for storing intermediate values
    mutable std::vector<double> inte;
    /// Maximum coordinate value found
    mutable double coord_max;
    /// Current coordinate value
    mutable double v;
    /// Index of the coordinate state
    mutable int m_state_index;
    /// Name of the coordinate state
    mutable std::string m_state_name;
    /// Function to compute power of coordinate values
    mutable std::function<double(const double&)> m_power_function;
    /// @}
};

} // namespace OpenSim

#endif // OPENSIM_MOCOMAXCOORDINATEGOAL_H
