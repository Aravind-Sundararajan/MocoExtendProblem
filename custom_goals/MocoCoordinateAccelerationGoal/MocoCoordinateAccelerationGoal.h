#ifndef OPENSIM_MOCOCOORDINATEACCELERATIONGOAL_H
#define OPENSIM_MOCOCOORDINATEACCELERATIONGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoCoordinateAccelerationGoal.h                                       *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 * -------------------------------------------------------------------------- */


#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "osimMocoCoordinateAccelerationGoalDLL.h"

#include <OpenSim/Common/TimeSeriesTable.h>

namespace OpenSim {

class OSIMMOCOCOORDINATEACCELERATIONGOAL_API MocoCoordinateAccelerationGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoCoordinateAccelerationGoal, MocoGoal);

public:
    MocoCoordinateAccelerationGoal() { constructProperties();}
    MocoCoordinateAccelerationGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoCoordinateAccelerationGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    // Public members to change the divide by displacement property
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    bool getDivideByDisplacement() const {
        return get_divide_by_displacement();
    }

    // Set the state names for coordinate acc minimization
    void setStateNames(const std::vector<std::string> refCoordNames) {
        std::cout << "Setting coordinate names from string " << std::endl;
        m_state_names = refCoordNames;
    }
    void setStateNames(const TimeSeriesTableVec3& ref) {
        m_state_names = ref.getColumnLabels();
    }
protected:
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    bool getSupportsEndpointConstraintImpl() const override { return false;}
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;

 private:
    OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
        "Divide by the model's displacement over the phase (default: "
        "false)");
    void constructProperties();

    mutable std::vector<int> m_sysYIndices;
    mutable std::vector<int> m_state_indices;
    // State for which we want to minimize acceleration. Column labels 
    // should be state variable paths, e.g., '/jointset/knee_r/knee_angle_r'
    mutable std::vector<std::string> m_state_names;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOCOORDINATEACCELERATIONGOAL_H
