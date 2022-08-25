#include "MocoOutputTrackingGoal.h"

using namespace OpenSim;

void MocoOutputTrackingGoal::constructProperties() {
    constructProperty_output_path("");
    constructProperty_divide_by_displacement(false);
    constructProperty_divide_by_mass(false);
    constructProperty_exponent(1);
    constructProperty_output_index(-1);
}

void MocoOutputTrackingGoal::initializeOnModelImpl(const Model& output) const {
    OPENSIM_THROW_IF_FRMOBJ(get_output_path().empty(), Exception,
            "No output_path provided.");
    std::string componentPath;
    std::string outputName;
    std::string channelName;
    std::string alias;
    AbstractInput::parseConnecteePath(
            get_output_path(), componentPath, outputName, channelName, alias);
    const auto& component = getModel().getComponent(componentPath);
    const auto* abstractOutput = &component.getOutput(outputName);

    OPENSIM_THROW_IF_FRMOBJ(get_output_index() < -1, Exception,
            "Invalid Output index provided.");
    m_minimizeVectorNorm = (get_output_index() == -1);

    if (dynamic_cast<const Output<double>*>(abstractOutput)) {
        m_data_type = Type_double;
        OPENSIM_THROW_IF_FRMOBJ(get_output_index() != -1, Exception,
                "An Output index was provided, but the Output is of type 'double'.")

    } else if (dynamic_cast<const Output<SimTK::Vec3>*>(abstractOutput)) {
        m_data_type = Type_Vec3;
        OPENSIM_THROW_IF_FRMOBJ(get_output_index() > 2, Exception,
                "The Output is of type 'SimTK::Vec3', but an Output index greater "
                "than 2 was provided.");
        m_index1 = get_output_index();

    } else if (dynamic_cast<const Output<SimTK::SpatialVec>*>(abstractOutput)) {
        m_data_type = Type_SpatialVec;
        OPENSIM_THROW_IF_FRMOBJ(get_output_index() > 5, Exception,
                "The Output is of type 'SimTK::SpatialVec', but an Output index "
                "greater than 5 was provided.");
        if (get_output_index() < 3) {
            m_index1 = 0;
            m_index2 = get_output_index();
        } else {
            m_index1 = 1;
            m_index2 = get_output_index() - 3;
        }

    } else {
        OPENSIM_THROW_FRMOBJ(Exception,
                "Data type of specified model output not supported.");
    }
    m_output.reset(abstractOutput);

    OPENSIM_THROW_IF_FRMOBJ(get_exponent() < 1, Exception,
            "Exponent must be 1 or greater.");
    int exponent = get_exponent();

    // The pow() function gives slightly different results than x * x. On Mac,
    // using x * x requires fewer solver iterations.
    if (exponent == 1) {
        m_power_function = [](const double& x) { return x; };
    } else if (exponent == 2) {
        m_power_function = [](const double& x) { return x * x; };
    } else {
        m_power_function = [exponent](const double& x) {
            return pow(std::abs(x), exponent);
        };
    }

    setRequirements(1, 1, m_output->getDependsOnStage());
}

void MocoOutputTrackingGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    getModel().getSystem().realize(input.state, m_output->getDependsOnStage());
    double value = 0;
    if (m_data_type == Type_double) {
        value = static_cast<const Output<double>*>(m_output.get())
                        ->getValue(input.state);

    } else if (m_data_type == Type_Vec3) {
        if (m_minimizeVectorNorm) {
            value = static_cast<const Output<SimTK::Vec3>*>(m_output.get())
                        ->getValue(input.state).norm();
        } else {
            value = static_cast<const Output<SimTK::Vec3>*>(m_output.get())
                        ->getValue(input.state)[m_index1];
        }

    } else if (m_data_type == Type_SpatialVec) {
        if (m_minimizeVectorNorm) {
            value = static_cast<const Output<SimTK::SpatialVec>*>(m_output.get())
                        ->getValue(input.state).norm();
        } else {
            value = static_cast<const Output<SimTK::SpatialVec>*>(m_output.get())
                        ->getValue(input.state)[m_index1][m_index2];
        }
    }

    integrand = m_power_function(value);
}

void MocoOutputTrackingGoal::calcGoalImpl(
        const MocoGoal::GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = (input.integral - get_reference_output()).normSqr();
    if (get_divide_by_displacement()) {
        cost[0] /=
                calcSystemDisplacement(input.initial_state, input.final_state);
    }
    if (get_divide_by_mass()) {
        cost[0] /= getModel().getTotalMass(input.initial_state);
    }
}