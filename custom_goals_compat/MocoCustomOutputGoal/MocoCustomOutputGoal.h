#ifndef OPENSIM_MOCOCUSTOMOUTPUTGOAL_H
#define OPENSIM_MOCOCUSTOMOUTPUTGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoCustomOutputGoal.h                                       *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "osimMocoCustomOutputGoalDLL.h"
#include <OpenSim/Common/TimeSeriesTable.h>

namespace OpenSim {

/** @brief Goal that minimizes a custom model output
 *
 * This goal minimizes any model output that can be specified by its absolute path,
 * integrated over the phase. The output can be a scalar value, a component of a
 * vector output, or the norm of a vector output.
 *
 * For vector outputs (including SpatialVec), you can either:
 * - Minimize a specific component by setting the output_index
 * - Minimize the vector norm by setting output_index to -1 (default)
 *
 * For SpatialVec outputs:
 * - Indices 0-2 refer to rotational components
 * - Indices 3-5 refer to translational components
 *
 * This goal can be useful for:
 * - Minimizing any custom quantity computed by the model
 * - Creating complex objective functions
 * - Targeting specific aspects of the motion
 *
 * The goal value can optionally be:
 * - Divided by the model's displacement over the phase
 * - Divided by the model's total mass
 * - Raised to a specified power using the exponent property
 */
class OSIMMOCOCUSTOMOUTPUTGOAL_API MocoCustomOutputGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoCustomOutputGoal, MocoGoal);

public:
    /** @name Constructors */
    /// @{
    /** Default constructor */
    MocoCustomOutputGoal() { constructProperties(); }

    /** Constructor with name
     * @param name The name of the goal */
    MocoCustomOutputGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }

    /** Constructor with name and weight
     * @param name The name of the goal
     * @param weight Weight for this goal term in the optimization */
    MocoCustomOutputGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }
    /// @}

    /** Set the absolute path to the output to minimize
     * @param path The output path in the model */
    void setOutputPath(std::string path) { set_output_path(std::move(path)); }
    
    /** Get the output path
     * @return The output path */
    const std::string& getOutputPath() const { return get_output_path(); }

    /** Set the exponent for the output value
     * @param exponent The exponent value */
    void setExponent(int exponent) { set_exponent(exponent); }
    
    /** Get the current exponent value
     * @return The exponent value */
    int getExponent() const { return get_exponent(); }

    /** Set the index for vector outputs
     * @param index The index to minimize (-1 for vector norm) */
    void setOutputIndex(int index) { set_output_index(index); }
    
    /** Get the output index
     * @return The output index */
    int getOutputIndex() const { return get_output_index(); }

    /** Set whether to divide by displacement
     * @param tf True to divide by displacement, false otherwise */
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    
    /** Get whether the goal is divided by displacement
     * @return True if divided by displacement, false otherwise */
    bool getDivideByDisplacement() const { return get_divide_by_displacement(); }

    /** Set whether to divide by model mass
     * @param mass True to divide by mass, false otherwise */
    void setDivideByMass(bool mass) { set_divide_by_mass(mass); }
    
    /** Get whether the goal is divided by mass
     * @return True if divided by mass, false otherwise */
    bool getDivideByMass() const { return get_divide_by_mass(); }

protected:
    /** @name Required implementations of virtual methods */
    /// @{
    /** Initialize the goal on the base model */
    void initializeOnModelBase() const;
    
    /** Calculate the output value for the current state
     * @param state The current state
     * @return The calculated output value */
    double calcOutputValue(const SimTK::State&) const;
    
    /** Apply the exponent to a value
     * @param value The value to modify
     * @return The value raised to the exponent */
    double setValueToExponent(double value) const;
    
    /** Get the stage that this goal depends on
     * @return The dependency stage */
    const SimTK::Stage& getDependsOnStage() const;

    /** Initialize the goal with the model */
    void initializeOnModelImpl(const Model&) const override;
    
    /** Calculate the integrand value for the cost function
     * @param state Input data for the current state
     * @param integrand Reference to store the calculated integrand value */
    void calcIntegrandImpl(
            const IntegrandInput& state, double& integrand) const override;
    
    /** Calculate the goal value 
     * @param input Input data containing the integral
     * @param values Vector to store the calculated values */
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& values) const override;
    
    /** Whether this goal supports endpoint constraint mode */
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    
    /** Get the default mode for this goal */
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    /// @}

private:
    /** @name Properties */
    /// @{
    OpenSim_DECLARE_PROPERTY(output_path, std::string,
            "The absolute path to the output in the model to use as the "
            "integrand for this goal.");

    OpenSim_DECLARE_PROPERTY(exponent, int,
            "The exponent applied to the output value in the integrand. "
            "The output can take on negative values in the integrand when the "
            "exponent is set to 1 (the default value). When the exponent is "
            "set to a value greater than 1, the absolute value function is "
            "applied to the output (before the exponent is applied), meaning "
            "that odd numbered exponents (greater than 1) do not take on "
            "negative values.");

    OpenSim_DECLARE_PROPERTY(output_index, int,
            "The index to the value to be minimized when a vector type "
            "Output is specified. For SpatialVec Outputs, indices 0, 1, "
            "and 2 refer to the rotational components and indices 3, 4, "
            "and 5 refer to the translational components. A value of -1 "
            "indicates to minimize the vector norm (default: -1).");

    OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
            "Divide by the model's displacement over the phase (default: "
            "false)");

    OpenSim_DECLARE_PROPERTY(divide_by_mass, bool,
            "Divide by the model's total mass (default: false)");
    /// @}

    /** Initialize the goal's properties */
    void constructProperties();

    /** @name Internal working variables */
    /// @{
    /// Type of data being processed
    mutable DataType m_data_type;
    /// Reference to the output being minimized
    mutable SimTK::ReferencePtr<const AbstractOutput> m_output;
    /// Function to compute power of values
    mutable std::function<double(const double&)> m_power_function;
    /// First index for vector components
    mutable int m_index1;
    /// Second index for vector components
    mutable int m_index2;
    /// Whether to minimize vector norm
    mutable bool m_minimizeVectorNorm;
    /// Stage at which the goal depends on the model
    mutable SimTK::Stage m_dependsOnStage = SimTK::Stage::Acceleration;
    /// @}

    /** @name Data type enumeration */
    /// @{
    /** Enumeration of possible output data types */
    enum DataType {
        Type_double,     ///< Scalar double value
        Type_Vec3,       ///< 3D vector
        Type_SpatialVec, ///< 6D spatial vector
    };
    /// @}
};

} // namespace OpenSim

#endif // OPENSIM_MOCOCUSTOMOUTPUTGOAL_H