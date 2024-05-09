#ifndef OPENSIM_MOCOZMPGOAL_H
#define OPENSIM_MOCOZMPGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoZMPGoal.h                                       *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 * -------------------------------------------------------------------------- */


#include <OpenSim/Moco/osimMoco.h>
#include "osimMocoZMPGoalDLL.h"

namespace OpenSim {

class OSIMMOCOZMPGOAL_API MocoZMPGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoZMPGoal, MocoGoal);

public:
    MocoZMPGoal() { constructProperties();}
    MocoZMPGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoZMPGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
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
    mutable SimTK::State _stateCopy;
    mutable Model m_model;
    
    void constructProperties();
};

} // namespace OpenSim

#endif // OPENSIM_MOCOZMPGOAL_H