#ifndef OPENSIM_MOCOACTIVATIONSQUAREDGOAL_H
#define OPENSIM_MOCOACTIVATIONSQUAREDGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoActivationSquaredGoal.h                                       *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi                                                     *
 * -------------------------------------------------------------------------- */


#include <OpenSim/Moco/osimMoco.h>
#include "osimMocoActivationSquaredGoalDLL.h"

namespace OpenSim {

class OSIMMOCOACTIVATIONSQUAREDGOAL_API MocoActivationSquaredGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoActivationSquaredGoal, MocoGoal);

public:
    MocoActivationSquaredGoal() { constructProperties();}
    MocoActivationSquaredGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoActivationSquaredGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
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

 private:
    OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
        "Divide by the model's displacement over the phase (default: "
        "false)");
    void constructProperties();

    mutable std::vector<int> m_act_indices;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOACTIVATIONSQUAREDGOAL_H
