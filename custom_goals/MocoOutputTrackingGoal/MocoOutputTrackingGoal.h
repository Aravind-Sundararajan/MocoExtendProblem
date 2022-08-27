#ifndef OPENSIM_MOCOOUTPUTTRACKINGGOAL_H
#define OPENSIM_MOCOOUTPUTTRACKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoOutputTrackingGoal.h                                          *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Moco/osimMoco.h>
#include "osimMocoOutputTrackingGoalDLL.h"

namespace OpenSim
{

    class OSIMMOCOOUTPUTTRACKINGGOAL_API MocoOutputTrackingGoal : public MocoGoal
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(MocoOutputTrackingGoal, MocoGoal);

    public:
        // Make the constructors for this class
        MocoOutputTrackingGoal() { constructProperties(); }
        MocoOutputTrackingGoal(std::string name) : MocoGoal(std::move(name))
        {
            constructProperties();
        }
        MocoOutputTrackingGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight)
        {
            constructProperties();
        }

        // Public methods to change and get the divide by displacement property
        void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
        bool getDivideByDisplacement() const { return get_divide_by_displacement(); }

        // Public methods to change and get the end-point goal value
        void setEndPointGoal(double end_point_goal) { set_end_point_goal(end_point_goal); }
        double getEndPointGoal() const { return get_end_point_goal(); }

    protected:
        Mode getDefaultModeImpl() const override { return Mode::Cost; }
        bool getSupportsEndpointConstraintImpl() const override { return true; }

        // Initialization function
        void initializeOnModelImpl(const Model &) const override;

        // Integration functions
        void calcIntegrandImpl(
            const IntegrandInput &input, double &integrand) const override;
        void calcGoalImpl(
            const GoalInput &input, SimTK::Vector &cost) const override;

    private:
        // Make the divide by displacement property
        OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
                                 "Divide by the model's displacement over the phase (default: "
                                 "false)");
        // Make the end_point_goal property
        OpenSim_DECLARE_PROPERTY(end_point_goal, double,
                                 "Target value for end-point goal (default: 0)");

        // Make a function to set the default values of these properties
        void constructProperties();

        // Make a private member that stores all the indices for muscle activations
        mutable std::vector<int> m_act_indices;
    };

} // namespace OpenSim

#endif // OPENSIM_MOCOOUTPUTTRACKINGGOAL_H
