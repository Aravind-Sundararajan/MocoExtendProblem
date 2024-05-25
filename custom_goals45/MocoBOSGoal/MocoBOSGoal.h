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

struct comData{
  std::map<SimTK::MobilizedBodyIndex, SimTK::Matrix> Fs;  //!<@brief  support force (spatial (6x1))
  std::map<SimTK::MobilizedBodyIndex, SimTK::Vec3> cop; //!<@brief  center of pressure (3x1)
  std::map<SimTK::MobilizedBodyIndex, SimTK::SpatialVec> force; //!<@brief  linear summed cop force and resultant moment (3x2)
};

class OSIMMOCOBOSGOAL_API MocoBOSGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoBOSGoal, MocoGoal);

public:
    MocoBOSGoal() { constructProperties();}
    MocoBOSGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoBOSGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
            }
	

    // Public member to set the exponent
    void setExponent(int ex) { set_exponent(ex); }
    bool getExponent() const { return get_exponent(); }

	/// Set the body frame associated with the left foot.
    void setLeftFootFrame(std::string left_foot) { set_left_foot_frame(std::move(left_foot)); }
    std::string getLeftFootFrame() const { 
		return get_left_foot_frame(); 
	}
    /// Set the body frame associated with the right foot.
    void setRightFootFrame(std::string right_foot) { set_right_foot_frame(std::move(right_foot)); }
    std::string getRightFootFrame() const { 
	    return get_right_foot_frame();
	}

protected:
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    bool getSupportsEndpointConstraintImpl() const override { return false;}
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    
    SimTK::Vec3 avg(const SimTK::Vec3& a, const SimTK::Vec3& b) const;
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
