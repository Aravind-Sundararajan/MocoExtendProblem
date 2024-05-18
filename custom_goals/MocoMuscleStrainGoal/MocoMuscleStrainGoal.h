#ifndef OPENSIM_MOCOMUSCLESTRAINGOAL_H
#define OPENSIM_MOCOMUSCLESTRAINGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoMuscleStrainGoal.h *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "osimMocoMuscleStrainGoalDLL.h"
#include <OpenSim/Moco/osimMoco.h>

namespace OpenSim {

class OSIMMOCOMUSCLESTRAINGOAL_API MocoMuscleStrainGoal : public MocoGoal {
  OpenSim_DECLARE_CONCRETE_OBJECT(MocoMuscleStrainGoal, MocoGoal);

public:
  MocoMuscleStrainGoal() { constructProperties(); }
  MocoMuscleStrainGoal(std::string name) : MocoGoal(std::move(name)) {
    constructProperties();
  }
  MocoMuscleStrainGoal(std::string name, double weight)
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

   // Make a private member that stores all the indices for muscle activations
   mutable std::vector<int> m_act_indices;
   mutable std::vector<double> m_Fiso;
   mutable std::vector<double> m_Lopt;    
    mutable std::vector<double> m_Vol; 
};

} // namespace OpenSim

#endif // OPENSIM_MOCOMUSCLESTRAINGOAL_H
