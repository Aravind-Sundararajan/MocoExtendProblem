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

/** This goal minimizes the acceleration of specified coordinates in the model.
 * The integrand is:
 * \f[
 * \sum_i |\ddot{q}_i|^p
 * \f]
 * where \f$ \ddot{q}_i \f$ are the coordinate accelerations and \f$ p \f$ is 
 * the exponent (default: 2).
 *
 * This goal can be useful for:
 * - Reducing rapid changes in movement
 * - Smoothing motion trajectories
 * - Minimizing jerk in the solution
 */
class OSIMMOCOCOORDINATEACCELERATIONGOAL_API MocoCoordinateAccelerationGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoCoordinateAccelerationGoal, MocoGoal);

public:
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

    /** Set the names of coordinates whose accelerations should be minimized
     * @param refCoordNames Vector of coordinate names. Each name should be a state 
     * variable path (e.g., '/jointset/knee_r/knee_angle_r') */
    void setStateNames(const std::vector<std::string> refCoordNames) {
        std::cout << "Setting coordinate names from string " << std::endl;
        m_state_names = refCoordNames;
    }

protected:
    /** @name Required implementations of virtual methods */
    /// @{
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    bool getSupportsEndpointConstraintImpl() const override { return false; }
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    /// @}

private:
    OpenSim_DECLARE_PROPERTY(exponent, int,
        "The exponent applied to the output value in the integrand. "
        "The output can take on negative values in the integrand when the "
        "exponent is set to 1 (the default value). When the exponent is "
        "set to a value greater than 1, the absolute value function is "
        "applied to the output (before the exponent is applied), meaning "
        "that odd numbered exponents (greater than 1) do not take on "
        "negative values.");

    void constructProperties();

    /** @name Internal working variables */
    /// @{
    /// Function to compute power of acceleration values
    mutable std::function<double(const double&)> m_power_function;
    /// System Y indices for all states
    mutable std::vector<int> m_sysYIndices;
    /// Indices of states we want to minimize acceleration for
    mutable std::vector<int> m_state_indices;
    /// Names of states for which we want to minimize acceleration
    mutable std::vector<std::string> m_state_names;
    /// @}
};

} // namespace OpenSim

#endif // OPENSIM_MOCOCOORDINATEACCELERATIONGOAL_H