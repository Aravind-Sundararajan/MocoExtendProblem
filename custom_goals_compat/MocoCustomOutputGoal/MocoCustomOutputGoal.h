#ifndef OPENSIM_MOCOCUSTOMOUTPUTGOAL_H
#define OPENSIM_MOCOCUSTOMOUTPUTGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoCustomOutputGoal.h                                       *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */


#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "osimMocoCustomOutputGoalDLL.h"

#include <OpenSim/Common/TimeSeriesTable.h>

namespace OpenSim {

class OSIMMOCOCUSTOMOUTPUTGOAL_API MocoCustomOutputGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoCustomOutputGoal, MocoGoal);

public:
    MocoCustomOutputGoal() { constructProperties();}
    MocoCustomOutputGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoCustomOutputGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }
	
    void setOutputPath(std::string path) { set_output_path(std::move(path)); }
    const std::string& getOutputPath() const { 
		return get_output_path(); 
	}
	
    void setExponent(int exponent) { set_exponent(exponent); }
    int getExponent() const { 
		return get_exponent(); 
	}
	
    void setOutputIndex(int index) { 
		set_output_index(index); 
	}
    int getOutputIndex() const { 
		return get_output_index(); 
		}
	
    // Public members to change the divide by displacement property
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    bool getDivideByDisplacement() const {
        return get_divide_by_displacement();
    }
	
    void setDivideByMass(bool mass) { set_divide_by_mass(mass); }
    bool getDivideByMass() const {
        return get_divide_by_mass();
    }

protected:
	void initializeOnModelBase() const;
	double calcOutputValue(const SimTK::State&) const;
    double setValueToExponent(double value) const;
    const SimTK::Stage& getDependsOnStage() const;
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& state, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& values) const override;
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::Cost;
    }

 private:
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
    mutable SimTK::Stage m_dependsOnStage = SimTK::Stage::Acceleration;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOCUSTOMOUTPUTGOAL_H