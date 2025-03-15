#ifndef OPENSIM_MOCOCOORDINATEACCELERATIONGOAL_H
#define OPENSIM_MOCOCOORDINATEACCELERATIONGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoCoordinateAccelerationGoal.h                                       *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "osimMocoCoordinateAccelerationGoalDLL.h"
#include <OpenSim/Common/TimeSeriesTable.h>

namespace OpenSim {

/** @brief Goal that minimizes the acceleration of specified coordinates
 *
 * This goal minimizes the acceleration of selected model coordinates,
 * integrated over the phase. This can help produce smoother, more natural motions
 * by penalizing rapid changes in coordinate velocities.
 *
 * The goal can be applied to any subset of model coordinates by specifying their
 * names. For each coordinate, the squared acceleration is computed and can be
 * raised to a specified power.
 *
 * This goal can be useful for:
 * - Generating smoother motions
 * - Reducing joint jerk
 * - Minimizing rapid changes in movement
 * - Improving numerical conditioning of the optimization
 *
 * The goal can optionally be divided by the total displacement of the model
 * during the phase to make the cost invariant to the distance traveled.
 */
class OSIMMOCOCOORDINATEACCELERATIONGOAL_API MocoCoordinateAccelerationGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoCoordinateAccelerationGoal, MocoGoal);

public:
    /** @name Constructors */
    /// @{
    /** Default constructor */
    MocoCoordinateAccelerationGoal() { constructProperties(); }

    /** Constructor with name
     * @param name The name of the goal */
    MocoCoordinateAccelerationGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }

    /** Constructor with name and weight
     * @param name The name of the goal
     * @param weight Weight for this goal term in the optimization */
    MocoCoordinateAccelerationGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }
    /// @}

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

    /** Set the coordinates to track for acceleration minimization
     * @param refCoordNames Vector of coordinate names to track
     * 
     * The coordinate names should be state variable paths, e.g.,
     * '/jointset/knee_r/knee_angle_r' */
    void setStateNames(const std::vector<std::string> refCoordNames) {
        std::cout << "Setting coordinate names from string " << std::endl;
        m_state_names = refCoordNames;
    }

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
    /// @}

    /** Initialize the goal's properties */
    void constructProperties();

    /** @name Internal working variables */
    /// @{
    /// System Y indices for state derivatives
    mutable std::vector<int> m_sysYIndices;
    /// Indices of states in the state vector
    mutable std::vector<int> m_state_indices;
    /// Names of coordinates to track
    mutable std::vector<std::string> m_state_names;
    /// Function to compute power of acceleration values
    mutable std::function<double(const double&)> m_power_function;
    /// @}
};

} // namespace OpenSim

#endif // OPENSIM_MOCOCOORDINATEACCELERATIONGOAL_H