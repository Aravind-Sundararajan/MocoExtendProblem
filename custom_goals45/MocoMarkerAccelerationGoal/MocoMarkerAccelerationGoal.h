#ifndef OPENSIM_MOCOMARKERACCELERATIONGOAL_H
#define OPENSIM_MOCOMARKERACCELERATIONGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoMarkerAccelerationGoal.h                                      *
 * -------------------------------------------------------------------------- *
  *                                                                           *
 * Author(s): Varun Joshi, Aravind Sundararajan                                                      *
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

protected:
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override; 

private:
    // PROPERTIES
    OpenSim_DECLARE_PROPERTY(exponent, int,
        "The exponent applied to the output value in the integrand. "
        "The output can take on negative values in the integrand when the "
        "exponent is set to 1 (the default value). When the exponent is "
        "set to a value greater than 1, the absolute value function is "
        "applied to the output (before the exponent is applied), meaning "
        "that odd numbered exponents (greater than 1) do not take on "
        "negative values.");
    OpenSim_DECLARE_PROPERTY(marker_name, std::string, "The name of the marker for this goal");

    void constructProperties();
    mutable std::function<double(const double&)> m_power_function;
    mutable SimTK::ReferencePtr<const Point> m_model_marker;
    
};

} // namespace OpenSim

#endif // OPENSIM_MOCOMARKERACCELERATIONGOAL_H
