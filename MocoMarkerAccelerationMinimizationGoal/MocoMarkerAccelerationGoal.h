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
#include "osimMocoMarkerAccelerationGoalDLL.h"
namespace OpenSim {

class Marker;

/** 
\section MocoMarkerAccelerationGoal
The absolute acceleration of a model marker summed over the x, y and z
integrated over the phase.
The name of the marker can be provided as a string.
*/
class OSIMMOCO_API MocoMarkerAccelerationGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoMarkerAccelerationGoal, MocoGoal);
public:
    MocoMarkerAccelerationGoal() { constructProperties(); }
    MocoMarkerAccelerationGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoMarkerAccelerationGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    void setMarkerName(std::string name) { set_marker_name(std::move(name)); }
    const std::string getMarkerName() const { return get_marker_name(); }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    void printDescriptionImpl() const override;
    // PROPERTIES
    OpenSim_DECLARE_PROPERTY(marker_name, std::string, "The name of the marker for this goal");

private:
    void constructProperties();
    mutable SimTK::ReferencePtr<const Marker> m_model_marker;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOMARKERACCELERATIONGOAL_H
