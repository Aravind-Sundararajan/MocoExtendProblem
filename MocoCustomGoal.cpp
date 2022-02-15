#include "MocoCustomGoal.h"

#include <OpenSim/Moco/MocoUtilities.h>

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

MocoCustomGoal::MocoCustomGoal() { constructProperties(); }

void MocoCustomGoal::constructProperties() {
    constructProperty_joint_path("");
    constructProperty_loads_frame("parent");
    constructProperty_expressed_in_frame_path("");
    constructProperty_reaction_measures();
    constructProperty_reaction_weights(MocoWeightSet());
}

void MocoCustomGoal::initializeOnModelImpl(const Model& model) const {

    // Cache the joint.
    OPENSIM_THROW_IF_FRMOBJ(get_joint_path().empty(), Exception,
            "Expected a joint path, but property joint_path is empty.");
    m_joint = &model.getComponent<Joint>(get_joint_path());

    m_denominator = model.getTotalMass(model.getWorkingState());
    const double gravityAccelMagnitude = model.get_gravity().norm();
    if (gravityAccelMagnitude > SimTK::SignificantReal) {
        m_denominator *= gravityAccelMagnitude;
    }

    // Get the frame from which the loads are computed.
    checkPropertyValueIsInSet(getProperty_loads_frame(), {"parent", "child"});
    if (get_loads_frame() == "parent") {
        m_isParentFrame = true;
    } else if (get_loads_frame() == "child") {
        m_isParentFrame = false;
    }

    // Get the expressed-in frame.
    if (get_expressed_in_frame_path().empty()) {
        if (m_isParentFrame) {
            m_frame = &m_joint->getParentFrame();
        } else {
            m_frame = &m_joint->getChildFrame();
        }
    } else {
        m_frame = &model.getComponent<Frame>(get_expressed_in_frame_path());
    }

    // If user provided no reaction measure names, then set all measures to
    // to be minimized. Otherwise, loop through user-provided measure names
    // and check that they are all accepted measures.
    std::vector<std::string> reactionMeasures;
    std::vector<std::string> allowedMeasures = {"moment-x", "moment-y",
            "moment-z", "force-x", "force-y", "force-z"};
    if (getProperty_reaction_measures().empty()) {
        reactionMeasures = allowedMeasures;
    } else {
        for (int i = 0; i < getProperty_reaction_measures().size(); ++i) {
            if (std::find(allowedMeasures.begin(), allowedMeasures.end(),
                        get_reaction_measures(i)) == allowedMeasures.end()) {

                OPENSIM_THROW_FRMOBJ(Exception,
                        "Reaction measure '{}' not recognized.",
                        get_reaction_measures(i));
            }
            reactionMeasures.push_back(get_reaction_measures(i));
        }
    }

    // Loop through all reaction measures to minimize and get the
    // corresponding SpatialVec indices and weights.
    for (const auto& measure : reactionMeasures) {
        if (measure == "moment-x") {
            m_measureIndices.push_back({0, 0});
        } else if (measure == "moment-y") {
            m_measureIndices.push_back({0, 1});
        } else if (measure == "moment-z") {
            m_measureIndices.push_back({0, 2});
        } else if (measure == "force-x") {
            m_measureIndices.push_back({1, 0});
        } else if (measure == "force-y") {
            m_measureIndices.push_back({1, 1});
        } else if (measure == "force-z") {
            m_measureIndices.push_back({1, 2});
        }

        double compWeight = 1.0;
        if (get_reaction_weights().contains(measure)) {
            compWeight = get_reaction_weights().get(measure).getWeight();
        }
        m_measureWeights.push_back(compWeight);
    }

    setRequirements(1, 1);
}

void MocoCustomGoal::calcIntegrandImpl(
        const IntegrandInput& input, SimTK::Real& integrand) const {

    getModel().realizeAcceleration(input.state);
    const auto& ground = getModel().getGround();

    // Compute the reaction loads on the parent or child frame.
    SimTK::SpatialVec reactionInGround;
    if (m_isParentFrame) {
        reactionInGround =
                m_joint->calcReactionOnParentExpressedInGround(input.state);
    } else {
        reactionInGround = m_joint->calcReactionOnChildExpressedInGround(input.state);
    }

    // Re-express the reactions into the proper frame and repackage into a new
    // SpatialVec.
    SimTK::Vec3 moment;
    SimTK::Vec3 force;
    if (m_frame.get() == &getModel().getGround()) {
        moment = reactionInGround[0];
        force = reactionInGround[1];
    } else {
        moment = ground.expressVectorInAnotherFrame(
                input.state, reactionInGround[0], *m_frame);
        force = ground.expressVectorInAnotherFrame(
                input.state, reactionInGround[1], *m_frame);
    }
    SimTK::SpatialVec reaction(moment, force);

    // Compute cost.
    integrand = 0;
    for (int i = 0; i < (int)m_measureIndices.size(); ++i) {
        const auto index = m_measureIndices[i];
        const double weight = m_measureWeights[i];
        integrand += weight * pow(reaction[index.first][index.second], 2);
    }
}

void MocoCustomGoal::printDescriptionImpl() const {
    log_cout("        joint path: ", get_joint_path());
    log_cout("        loads frame: ", get_loads_frame());
    log_cout("        expressed: ", get_expressed_in_frame_path());

    std::vector<std::string> measures(getProperty_reaction_measures().size());
    for (int i = 0; i < (int)measures.size(); i++) {
        measures[i] = get_reaction_measures(i);
    }
    log_cout("        reaction measures: {}", fmt::join(measures, ", "));

    log_cout("        reaction weights: {}", fmt::join(m_measureWeights, ", "));
}