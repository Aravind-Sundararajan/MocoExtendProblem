#ifndef OPENSIM_MOCOWALKINGGOAL_H
#define OPENSIM_MOCOWALKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoWalkingGoal.h                                       *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 * -------------------------------------------------------------------------- */


#include <OpenSim/Moco/osimMoco.h>
#include "osimMocoWalkingGoalDLL.h"

namespace OpenSim {

class OSIMMOCOWALKINGGOAL_API MocoWalkingGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoWalkingGoal, MocoGoal);

public:
    MocoWalkingGoal() { constructProperties();}
    MocoWalkingGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoWalkingGoal(std::string name, double weight)
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
};

} // namespace OpenSim

#endif // OPENSIM_MOCOWALKINGGOAL_H
