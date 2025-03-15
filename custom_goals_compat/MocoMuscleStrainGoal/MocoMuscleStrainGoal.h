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

/** @brief Goal that minimizes muscle strain during motion
 *
 * This goal minimizes the strain experienced by muscles during the motion,
 * integrated over the phase. The strain is computed based on the muscle's
 * current length relative to its optimal fiber length.
 *
 * The goal accounts for:
 * - Muscle fiber lengths
 * - Optimal fiber lengths
 * - Muscle volumes
 * - Maximum isometric forces
 *
 * This goal can be useful for:
 * - Preventing excessive muscle stretching
 * - Reducing risk of muscle injury
 * - Generating more physiologically realistic motions
 * - Maintaining muscle lengths within normal operating ranges
 *
 * The strain values can be raised to a specified power using the exponent
 * property, and can optionally be divided by the total displacement of the
 * model during the phase to make the cost invariant to the distance traveled.
 */
class OSIMMOCOMUSCLESTRAINGOAL_API MocoMuscleStrainGoal : public MocoGoal {
  OpenSim_DECLARE_CONCRETE_OBJECT(MocoMuscleStrainGoal, MocoGoal);

public:
  /** @name Constructors */
  /// @{
  /** Default constructor */
  MocoMuscleStrainGoal() { constructProperties(); }

  /** Constructor with name
   * @param name The name of the goal */
  MocoMuscleStrainGoal(std::string name) : MocoGoal(std::move(name)) {
    constructProperties();
  }

  /** Constructor with name and weight
   * @param name The name of the goal
   * @param weight Weight for this goal term in the optimization */
  MocoMuscleStrainGoal(std::string name, double weight)
      : MocoGoal(std::move(name), weight) {
    constructProperties();
  }
  /// @}

  /** Set the exponent for the strain terms
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

  /** @name Internal working variables */
  /// @{
  /// Names of forces used in calculations
  mutable std::vector<std::string> m_force_names;
  /// Function to compute power of values
  mutable std::function<double(const double&)> m_power_function;
  /// Indices of muscle activation states
  mutable std::vector<int> m_act_indices;
  /// Maximum isometric forces for each muscle
  mutable std::vector<double> m_Fiso;
  /// Optimal fiber lengths for each muscle
  mutable std::vector<double> m_Lopt;
  /// Muscle volumes
  mutable std::vector<double> m_Vol;
  /// @}
};

} // namespace OpenSim

#endif // OPENSIM_MOCOMUSCLESTRAINGOAL_H
