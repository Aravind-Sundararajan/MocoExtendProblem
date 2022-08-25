#ifndef OPENSIM_MOCOOUTPUTTRACKINGGOAL_H
#define OPENSIM_MOCOOUTPUTTRACKINGGOAL_H

#include "MocoGoal.h"

namespace OpenSim {

/** This goal allows you to use model Outputs of type double, SimTK::Vec3, and
SimTK::SpatialVec in the integrand of a goal. By default, when using vector
type Outputs, the norm of the vector is minimized, but you can also minimize
a specific element of a vector Output via `setOutputIndex()`. You can also
specify the exponent of the value in the integrand via `setExponent()`.
@ingroup mocogoal */
class OSIMMOCO_API MocoOutputTrackingGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoOutputTrackingGoal, MocoGoal);

public:
    MocoOutputTrackingGoal() { constructProperties(); }
    MocoOutputTrackingGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoOutputTrackingGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /// Set the desired integral of the output
    void setReferenceOutput(double refOutput) {
        set_reference_output(std::move(refOutput));
    }

    /** Set the absolute path to the output in the model to use as the integrand
    for this goal. The format is "/path/to/component|output_name". */
    void setOutputPath(std::string path) { set_output_path(std::move(path)); }
    const std::string& getOutputPath() const { return get_output_path(); }

    /** Set if the goal should be divided by the displacement of the system's
    center of mass over the phase. */
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    bool getDivideByDisplacement() const {
        return get_divide_by_displacement();
    }

    /** Set if the goal should be divided by the total mass of the model. */
    void setDivideByMass(bool tf) { set_divide_by_mass(tf); }
    bool getDivideByMass() const {
        return get_divide_by_mass();
    }

    /** Set the exponent applied to the output value in the integrand. This
    exponent is applied when minimizing the norm of a vector type output. */
    void setExponent(int exponent) { set_exponent(exponent); }
    int getExponent() const { return get_exponent(); }

    /** Set the index to the value to be minimized when a vector type
    Output is specified. For SpatialVec Outputs, indices 0, 1, and 2
    refer to the rotational components and indices 3, 4, and 5 refer
    to the translational components. A value of -1 indicates to
    minimize the vector norm. If an index for a type double Output
    is provided, an exception is thrown. */
    void setOutputIndex(int index) { set_output_index(index); }
    int getOutputIndex() const { return get_output_index(); }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& state, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;

private:
    OpenSim_DECLARE_PROPERTY(reference_output, double,
        "The desired final value of the integral as double");
    OpenSim_DECLARE_PROPERTY(output_path, std::string,
            "The absolute path to the output in the model to use as the "
            "integrand for this goal.");
    OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
            "Divide by the model's displacement over the phase (default: "
            "false)");
    OpenSim_DECLARE_PROPERTY(divide_by_mass, bool,
            "Divide by the model's total mass (default: false)");
    OpenSim_DECLARE_PROPERTY(exponent, int,
            "The exponent applied to the output value in the integrand "
            "(default: 1).");
    OpenSim_DECLARE_PROPERTY(output_index, int,
            "The index to the value to be minimized when a vector type "
            "Output is specified. For SpatialVec Outputs, indices 0, 1, "
            "and 2 refer to the rotational components and indices 3, 4, "
            "and 5 refer to the translational components. A value of -1 "
            "indicates to minimize the vector norm (default: -1).");
    void constructProperties();

    enum DataType {
        Type_double,
        Type_Vec3,
        Type_SpatialVec,
    };
    mutable DataType m_data_type;
    mutable SimTK::ReferencePtr<const AbstractOutput> m_output;
    mutable std::function<double(const double&)> m_power_function;
    mutable int m_index1;
    mutable int m_index2;
    mutable bool m_minimizeVectorNorm;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOOUTPUTTRACKINGGOAL_H