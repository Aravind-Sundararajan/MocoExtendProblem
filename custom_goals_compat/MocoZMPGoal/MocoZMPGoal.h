#ifndef OPENSIM_MOCOZMPGOAL_H
#define OPENSIM_MOCOZMPGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoZMPGoal.h                                                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoZMPGoalDLL.h"
#include <OpenSim/Moco/osimMoco.h>

namespace OpenSim {

class OSIMMOCOZMPGOAL_API MocoZMPGoal : public MocoGoal {
  OpenSim_DECLARE_CONCRETE_OBJECT(MocoZMPGoal, MocoGoal);

public:
  MocoZMPGoal() { constructProperties(); }
  MocoZMPGoal(std::string name) : MocoGoal(std::move(name)) {
    constructProperties();
  }
  MocoZMPGoal(std::string name, double weight)
      : MocoGoal(std::move(name), weight) {
    constructProperties();
  }

  // Public members to change the divide by displacement property
  void setExponent(int ex) { set_exponent(ex); }
  bool getExponent() const { return get_exponent(); }

  // Public members to change the divide by displacement property
  void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
  bool getDivideByDisplacement() const { return get_divide_by_displacement(); }

protected:
  Mode getDefaultModeImpl() const override { return Mode::Cost; }
  bool getSupportsEndpointConstraintImpl() const override { return false; }
  void initializeOnModelImpl(const Model &) const override;
  void calcIntegrandImpl(const IntegrandInput &input,
                         double &integrand) const override;
  void calcGoalImpl(const GoalInput &input, SimTK::Vector &cost) const override;

  SimTK::Matrix FlattenSpatialVec(const SimTK::SpatialVec &S) const;

private:
  OpenSim_DECLARE_PROPERTY(
      divide_by_displacement, bool,
      "Divide by the model's displacement over the phase (default: "
      "false)");
  OpenSim_DECLARE_PROPERTY(
      exponent, int,
      "The exponent applied to the output value in the integrand. "
      "The output can take on negative values in the integrand when the "
      "exponent is set to 1 (the default value). When the exponent is "
      "set to a value greater than 1, the absolute value function is "
      "applied to the output (before the exponent is applied), meaning "
      "that odd numbered exponents (greater than 1) do not take on "
      "negative values.");
  void constructProperties();
  mutable std::vector<std::string> m_force_names;
  mutable std::function<double(const double &)> m_power_function;
  mutable OpenSim::Model m_model;

};

} // namespace OpenSim

#endif // OPENSIM_MOCOZMPGOAL_H
