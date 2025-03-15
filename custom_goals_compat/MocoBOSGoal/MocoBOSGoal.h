#ifndef OPENSIM_MOCOBOSGOAL_H
#define OPENSIM_MOCOBOSGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoBOSGoal.h                                                     *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Moco/osimMoco.h>
#include "osimMocoBOSGoalDLL.h"

namespace OpenSim {

/** @brief Structure containing data related to center of mass and support forces
 *
 * This structure stores maps that relate mobilized body indices to their
 * respective support forces, centers of pressure, and resultant forces/moments.
 */
struct comData {
    std::map<SimTK::MobilizedBodyIndex, SimTK::Matrix> Fs;     //!< Support force (spatial 6x1)
    std::map<SimTK::MobilizedBodyIndex, SimTK::Vec3> cop;      //!< Center of pressure (3x1)
    std::map<SimTK::MobilizedBodyIndex, SimTK::SpatialVec> force; //!< Linear summed COP force and resultant moment (3x2)
};

/** @brief Goal that minimizes the deviation of the base of support (BOS) from a target
 *
 * This goal minimizes the deviation of the model's base of support from a desired
 * target location, integrated over the phase. The base of support is calculated
 * using the contact forces and center of pressure between the feet and the ground.
 *
 * This goal can be useful for:
 * - Improving balance and stability in walking simulations
 * - Controlling foot placement and weight transfer
 * - Minimizing risk of falls
 *
 * The goal requires specification of frames for both feet to properly calculate
 * the base of support. The deviation can optionally be divided by the total 
 * displacement of the model during the phase to make the cost invariant to the 
 * distance traveled.
 */
class OSIMMOCOBOSGOAL_API MocoBOSGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoBOSGoal, MocoGoal);

public:
    /** @name Constructors */
    /// @{
    /** Default constructor */
    MocoBOSGoal() { constructProperties(); }

    /** Constructor with name
     * @param name The name of the goal */
    MocoBOSGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }

    /** Constructor with name and weight
     * @param name The name of the goal
     * @param weight Weight for this goal term in the optimization */
    MocoBOSGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }
    /// @}

    /** Set the exponent for the deviation term
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

    /** Set the body frame for the left foot
     * @param left_foot Name of the left foot frame */
    void setLeftFootFrame(std::string left_foot) { 
        set_left_foot_frame(std::move(left_foot)); 
    }
    
    /** Get the name of the left foot frame
     * @return Name of the left foot frame */
    std::string getLeftFootFrame() const { return get_left_foot_frame(); }

    /** Set the body frame for the right foot
     * @param right_foot Name of the right foot frame */
    void setRightFootFrame(std::string right_foot) { 
        set_right_foot_frame(std::move(right_foot)); 
    }
    
    /** Get the name of the right foot frame
     * @return Name of the right foot frame */
    std::string getRightFootFrame() const { return get_right_foot_frame(); }

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

    /** Calculate the average of two 3D vectors
     * @param a First vector
     * @param b Second vector
     * @return Average vector */
    SimTK::Vec3 avg(const SimTK::Vec3& a, const SimTK::Vec3& b) const;
    
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

    OpenSim_DECLARE_PROPERTY(left_foot_frame, std::string,
            "The model frame associated with the left foot.");

    OpenSim_DECLARE_PROPERTY(right_foot_frame, std::string,
            "The model frame associated with the right foot.");
    /// @}

    /** Initialize the goal's properties */
    void constructProperties();

    /** @name Internal working variables */
    /// @{
    /// Names of forces used in calculations
    mutable std::vector<std::string> m_force_names;
    /// Function to compute power of values
    mutable std::function<double(const double&)> m_power_function;
    /// Reference to the left foot frame
    mutable SimTK::ReferencePtr<const Body> m_left_foot_frame;
    /// Reference to the right foot frame
    mutable SimTK::ReferencePtr<const Body> m_right_foot_frame;
    /// @}
};

} // namespace OpenSim

#endif // OPENSIM_MOCOBOSGOAL_H
