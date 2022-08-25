#ifndef OPENSIM_MOCOMARKERACCELERATIONGOAL_H
#define OPENSIM_MOCOMARKERACCELERATIONGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoMarkerAccelerationGoal.h                                      *
 * -------------------------------------------------------------------------- *
  *                                                                           *
 * Author(s): Varun Joshi                                                     *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "osimMocoMarkerAccelerationGoalDLL.h"

namespace OpenSim {

/** 
\section MocoMarkerAccelerationGoal
The absolute acceleration of a model marker summed over the x, y and z
integrated over the phase.
The name of the marker can be provided as a string.
*/
class OSIMMOCOMARKERACCELERATIONGOAL_API MocoMarkerAccelerationGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoMarkerAccelerationGoal, MocoGoal);
public:
    MocoMarkerAccelerationGoal() { constructProperties(); }
    MocoMarkerAccelerationGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoMarkerAccelerationGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        log_cout("Constructed class");
        constructProperties();
    }

    // Public member to set the name of the marker
    void setMarkerName(std::string name) { set_marker_name(std::move(name)); }
    //std::string getMarkerName() const { return get_marker_name(); }

    // Public members to change the divide by displacement property
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    bool getDivideByDisplacement() const { return get_divide_by_displacement(); }

protected:
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override; 

private:
    // PROPERTIES
    OpenSim_DECLARE_PROPERTY(marker_name, std::string, "The name of the marker for this goal");
    OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
        "Divide by the model's displacement over the phase (default: "
        "false)");
    void constructProperties();

    mutable SimTK::ReferencePtr<const Point> m_model_marker;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOMARKERACCELERATIONGOAL_H
