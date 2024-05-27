/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCustomOutputGoal.cpp                                              *
 * -------------------------------------------------------------------------- *
  *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoCustomOutputGoal.h"

using namespace OpenSim;

double MocoCustomOutputGoal::setValueToExponent(double value) const {
	return m_power_function(value);
}

const SimTK::Stage& MocoCustomOutputGoal::getDependsOnStage() const {
	return m_dependsOnStage;
}

void MocoCustomOutputGoal::constructProperties() {
    constructProperty_output_path("");
    constructProperty_exponent(1);
    constructProperty_output_index(-1);
    constructProperty_divide_by_displacement(false);
    constructProperty_divide_by_mass(false);
}

void MocoCustomOutputGoal::initializeOnModelBase() const {
    std::string componentPath, outputName, channelName, alias;
    AbstractInput::parseConnecteePath(
            get_output_path(), componentPath, outputName, channelName, alias);
    const auto& component = getModel().getComponent(componentPath);
    const auto* abstractOutput = &component.getOutput(outputName);
    m_minimizeVectorNorm = (get_output_index() == -1);

    if (dynamic_cast<const Output<double>*>(abstractOutput)) {
        m_data_type = Type_double;

    } else if (dynamic_cast<const Output<SimTK::Vec3>*>(abstractOutput)) {
        m_data_type = Type_Vec3;
        m_index1 = get_output_index();

    } else if (dynamic_cast<const Output<SimTK::SpatialVec>*>(abstractOutput)) {
        m_data_type = Type_SpatialVec;
        if (get_output_index() < 3) {
            m_index1 = 0;
            m_index2 = get_output_index();
        } else {
            m_index1 = 1;
            m_index2 = get_output_index() - 3;
        }

    } else {
		//
    }
    m_output.reset(abstractOutput);

  int exponent = get_exponent();

  // The pow() function gives slightly different results than x * x. On Mac,
  // using x * x requires fewer solver iterations.
  if (exponent == 1) {
    m_power_function = [](const double &x) { return std::abs(x); };
  } else if (exponent == 2) {
    m_power_function = [](const double &x) { return x * x; };
  } else {
    m_power_function = [exponent](const double &x) {
      return pow(std::abs(x), exponent);
    };
  }

    // Set the "depends-on stage", the SimTK::Stage we must realize to
    // in order to calculate values from this output.
    m_dependsOnStage = m_output->getDependsOnStage();
}

double MocoCustomOutputGoal::calcOutputValue(const SimTK::State& state) const {
    getModel().getSystem().realize(state, m_output->getDependsOnStage());

    double value = 0;
    if (m_data_type == Type_double) {
        value = static_cast<const Output<double>*>(m_output.get())
                        ->getValue(state);

    } else if (m_data_type == Type_Vec3) {
        if (m_minimizeVectorNorm) {
            value = static_cast<const Output<SimTK::Vec3>*>(m_output.get())
                        ->getValue(state).norm();
        } else {
            value = static_cast<const Output<SimTK::Vec3>*>(m_output.get())
                        ->getValue(state)[m_index1];
        }

    } else if (m_data_type == Type_SpatialVec) {
        if (m_minimizeVectorNorm) {
            value = static_cast<const Output<SimTK::SpatialVec>*>(m_output.get())
                        ->getValue(state).norm();
        } else {
            value = static_cast<const Output<SimTK::SpatialVec>*>(m_output.get())
                        ->getValue(state)[m_index1][m_index2];
        }
    }

    return value;
}

void MocoCustomOutputGoal::initializeOnModelImpl(const Model& output) const {
    initializeOnModelBase();
    setRequirements(1, 1, getDependsOnStage());
}

void MocoCustomOutputGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    integrand = setValueToExponent(calcOutputValue(input.state));
}

void MocoCustomOutputGoal::calcGoalImpl(
        const MocoGoal::GoalInput& input, SimTK::Vector& values) const {
    values[0] = input.integral;
    if (get_divide_by_displacement()) {
        values[0] /=
                calcSystemDisplacement(input.initial_state, input.final_state);
    }
    if (get_divide_by_mass()) {
        values[0] /= getModel().getTotalMass(input.initial_state);
    }
}