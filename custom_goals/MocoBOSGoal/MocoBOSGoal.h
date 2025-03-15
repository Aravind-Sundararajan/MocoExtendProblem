#ifndef OPENSIM_MOCOBOSGOAL_H
#define OPENSIM_MOCOBOSGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoBOSGoal.h                                                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */


#include <OpenSim/Moco/osimMoco.h>
#include "osimMocoBOSGoalDLL.h"

namespace OpenSim {

/** @brief Structure containing data related to center of pressure and forces
 * for body segments */
struct comData{
  std::map<SimTK::MobilizedBodyIndex, SimTK::Matrix> Fs;  //!<@brief  support force (spatial (6x1))
  std::map<SimTK::MobilizedBodyIndex, SimTK::Vec3> cop; //!<@brief  center of pressure (3x1)
  std::map<SimTK::MobilizedBodyIndex, SimTK::SpatialVec> force; //!<@brief  linear summed cop force and resultant moment (3x2)
};

/** This goal minimizes the distance between the body's center of mass projection
 * and the center of the base of support defined by the foot frames. The goal helps
 * maintain balance during movement optimization.
 *
 * The base of support is calculated using the specified left and right foot frames.
 * The goal can be weighted and the distance measure can be modified using an
 * exponent parameter.
 */
class OSIMMOCOBOSGOAL_API MocoBOSGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoBOSGoal, MocoGoal);

public:
    /** Default constructor */
    MocoBOSGoal() { constructProperties();}
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
	

    /** Set the exponent applied to the distance measure
     * @param ex The exponent value. For ex > 1, absolute value is applied first */
    void setExponent(int ex) { set_exponent(ex); }
    /** Get the current exponent value
     * @return The exponent applied to the distance measure */
    bool getExponent() const { return get_exponent(); }

    /** Set the frame associated with the left foot
     * @param left_foot Name of the left foot frame in the model */
    void setLeftFootFrame(std::string left_foot) { set_left_foot_frame(std::move(left_foot)); }
    /** Get the name of the left foot frame
     * @return Name of the left foot frame */
    std::string getLeftFootFrame() const { 
		return get_left_foot_frame(); 
	}
    /** Set the frame associated with the right foot
     * @param right_foot Name of the right foot frame in the model */
    void setRightFootFrame(std::string right_foot) { set_right_foot_frame(std::move(right_foot)); }
    /** Get the name of the right foot frame
     * @return Name of the right foot frame */
    std::string getRightFootFrame() const { 
	    return get_right_foot_frame();
	}

protected:
    /** @name Required implementations of virtual methods */
    /// @{
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    bool getSupportsEndpointConstraintImpl() const override { return false;}
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    /// @}
    
    /** Calculate the average of two 3D vectors
     * @param a First vector
     * @param b Second vector
     * @return Average vector */
    SimTK::Vec3 avg(const SimTK::Vec3& a, const SimTK::Vec3& b) const;
    /** Convert a spatial vector to a matrix representation
     * @param S Spatial vector to flatten
     * @return Matrix representation */
    SimTK::Matrix FlattenSpatialVec(const SimTK::SpatialVec& S) const;


 private:
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
    void constructProperties();
	mutable std::vector<std::string> m_force_names;
    mutable SimTK::ReferencePtr<const Body> m_left_foot_frame;
    mutable SimTK::ReferencePtr<const Body> m_right_foot_frame;
    mutable std::function<double(const double &)> m_power_function;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOBOSGOAL_H
