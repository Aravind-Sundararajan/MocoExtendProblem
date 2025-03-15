#ifndef OPENSIM_MOCOCOPGOAL_H
#define OPENSIM_MOCOCOPGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoCOPGoal.h                                                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */


#include <OpenSim/Moco/osimMoco.h>
#include "osimMocoCOPGoalDLL.h"

namespace OpenSim {

/** @brief Structure containing data related to support forces and center of pressure
 *
 * This structure stores maps that relate mobilized body indices to their
 * respective support forces, centers of pressure, and resultant forces/moments.
 */
struct supportData{
  std::map<SimTK::MobilizedBodyIndex, SimTK::Matrix> Fs;  //!<@brief  support force (spatial (6x1))
  std::map<SimTK::MobilizedBodyIndex, SimTK::Vec3> cop; //!<@brief  center of pressure (3x1)
  std::map<SimTK::MobilizedBodyIndex, SimTK::SpatialVec> force; //!<@brief  linear summed cop force and resultant moment (3x2)
};

/** @brief Goal that minimizes the deviation of the center of pressure (COP) from a target
 *
 * This goal minimizes the deviation of the model's center of pressure from a desired
 * target location, integrated over the phase. The center of pressure is calculated
 * using the contact forces between the model and the ground.
 *
 * This goal can be useful for:
 * - Controlling weight transfer during stance
 * - Improving balance and stability
 * - Generating more natural gait patterns
 * - Reducing excessive pressure points
 *
 * The goal can optionally be divided by the total displacement of the model
 * during the phase to make the cost invariant to the distance traveled.
 */
class OSIMMOCOCOPGOAL_API MocoCOPGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoCOPGoal, MocoGoal);

public:
    /** @name Constructors */
    /// @{
    /** Default constructor */
    MocoCOPGoal() { constructProperties();}

    /** Constructor with name
     * @param name The name of the goal */
    MocoCOPGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }

    /** Constructor with name and weight
     * @param name The name of the goal
     * @param weight Weight for this goal term in the optimization */
    MocoCOPGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }
    /// @}
	
    /** Set the exponent for the deviation term
     * @param ex The exponent value */
    void setExponent(int ex) { set_exponent(ex); }
    
    /** Get the current exponent value
     * @return The exponent value */
    bool getExponent() const {
        return get_exponent();
    }
	
    /** Set whether to divide by displacement
     * @param tf True to divide by displacement, false otherwise */
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    
    /** Get whether the goal is divided by displacement
     * @return True if divided by displacement, false otherwise */
    bool getDivideByDisplacement() const {
        return get_divide_by_displacement();
    }

protected:
    /** @name Required implementations of virtual methods */
    /// @{
    /** Get the default mode for this goal */
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    
    /** Whether this goal supports endpoint constraint mode */
    bool getSupportsEndpointConstraintImpl() const override { return false;}

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
    
    /** Convert a spatial vector to a matrix representation
     * @param S Spatial vector to flatten
     * @return Matrix representation */
    SimTK::Matrix FlattenSpatialVec(const SimTK::SpatialVec& S) const;
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
	mutable std::vector<std::string> m_force_names;
	mutable std::function<double(const double&)> m_power_function;
   
};

} // namespace OpenSim

#endif // OPENSIM_MOCOCOPGOAL_H
