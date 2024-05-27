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

struct supportData{
  std::map<SimTK::MobilizedBodyIndex, SimTK::Matrix> Fs;  //!<@brief  support force (spatial (6x1))
  std::map<SimTK::MobilizedBodyIndex, SimTK::Vec3> cop; //!<@brief  center of pressure (3x1)
  std::map<SimTK::MobilizedBodyIndex, SimTK::SpatialVec> force; //!<@brief  linear summed cop force and resultant moment (3x2)
};

class OSIMMOCOCOPGOAL_API MocoCOPGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoCOPGoal, MocoGoal);

public:
    MocoCOPGoal() { constructProperties();}
    MocoCOPGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoCOPGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }
	
    // Public members to change the divide by displacement property
    void setExponent(int ex) { set_exponent(ex); }
    bool getExponent() const {
        return get_exponent();
    }
	
    // Public members to change the divide by displacement property
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    bool getDivideByDisplacement() const {
        return get_divide_by_displacement();
    }

protected:
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    bool getSupportsEndpointConstraintImpl() const override { return false;}
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    
    SimTK::Matrix FlattenSpatialVec(const SimTK::SpatialVec& S) const;


 private:

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
    void constructProperties();
	mutable std::vector<std::string> m_force_names;
	mutable std::function<double(const double&)> m_power_function;
   
};

} // namespace OpenSim

#endif // OPENSIM_MOCOCOPGOAL_H
