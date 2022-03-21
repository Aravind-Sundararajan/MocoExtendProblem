/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoOutputTrackingGoal.cpp                                   *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoOutputTrackingGoal.h"
#include <OpenSim/Actuators/ModelOperators.h>
using namespace OpenSim;

void MocoOutputTrackingGoal::constructProperties() {
    constructProperty_divide_by_displacement(false);
}

void MocoOutputTrackingGoal::initializeOnModelImpl(const Model& model) const {
    setRequirements(1, 1);
     ModelProcessor m_p(model);
     m_p.append(ModOpRemoveMuscles());
     m_model = m_p.process();
     for (int f =0;f<m_model.updForceSet().getSize();f++) {
        m_model.updForceSet().remove(f);
     };
     m_model.finalizeFromProperties();
     m_model.finalizeConnections();
     _stateCopy = m_model.initSystem();
}

void MocoOutputTrackingGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
 	SimTK::Vec3 PelvisInGround(0.0, 0.0, 0.0);
    SimTK::Vec3 COMinGround(0.0, 0.0, 0.0);
	SimTK::Vec3 pelvis_moment(0.0, 0.0, 0.0);
	SimTK::Vec3 pelvis_force(0.0, 0.0, 0.0);
    SimTK::Vec3 PelvisCoM(0.0, 0.0, 0.0);
	SimTK::Vec3 OutputTracking_moment(0.0, 0.0, 0.0);
    SimTK::Vec3 OutputTracking_force(0.0, 0.0, 0.0);
    SimTK::Vec3 OutputTrackingout(0.0, 0.0, 0.0);
    SimTK::Vector residualMobilityForces;
    SimTK::Vector knownUdot;
    SimTK::State& state = _stateCopy;
    state.updQ() = input.state.getQ();
    state.updU() = input.state.getU();
    state.updUDot() = input.state.getUDot();
    m_model.realizeDynamics(state);


	COMinGround =m_model.calcMassCenterPosition(state);
	COMinGround[1] = 0.0;
    //std::cout << "project COM to Ground: " << COMinGround << std::endl;
	// Calculate OutputTracking
		
 	SimTK::Vector_<SimTK::SpatialVec> appliedBodyForces = m_model.getMultibodySystem().getRigidBodyForces(state, SimTK::Stage::Dynamics);
	//appliedBodyForces.dump("All Applied Body Forces");
    
    SimTK::Vector appliedMobilityForces(state.getU().size());
	// Removing mobility forces
    appliedMobilityForces *=0;
    //std::cout << "instantiate applied mobility forces: " << appliedMobilityForces << std::endl;
	
	// Results of the inverse dynamics for the generalized forces to satisfy accelerations
	
	knownUdot = state.getUDot();
    //std::cout << "known UDot: " << knownUdot << std::endl;
 
	// Perform inverse dynamics
	m_model.getMultibodySystem().getMatterSubsystem().calcResidualForceIgnoringConstraints(state, appliedMobilityForces, appliedBodyForces, knownUdot, residualMobilityForces);
	
	/* Get model free floating joint (Pelvis Joint or the first joint conncted to the ground)*/
	SimTK::SpatialVec equivalentBodyForcesAtPelvis = m_model.getJointSet()[0].calcEquivalentSpatialForce(state, residualMobilityForces);
	
	for(int n=0;n<3;n++){ 
	pelvis_moment[n] = equivalentBodyForcesAtPelvis[0][n];
	pelvis_force[n] = equivalentBodyForcesAtPelvis[1][n];
	}
	
	
	PelvisCoM = m_model.getBodySet().get(0).getMassCenter();
	PelvisInGround = m_model.getBodySet().get(0).findStationLocationInGround(state, PelvisCoM);
	//std::cout << "pelvis in ground: " << PelvisInGround << std::endl;

 	OutputTracking_moment = pelvis_moment + PelvisInGround % pelvis_force;
	
	OutputTracking_force[0] = pelvis_force[0];
	OutputTracking_force[1] = pelvis_force[1];
	OutputTracking_force[2] = pelvis_force[2];
	
	OutputTrackingout[0] = OutputTracking_moment[2]/OutputTracking_force[1];
	OutputTrackingout[1] = 0.0;
	OutputTrackingout[2] = -1.0 * (OutputTracking_moment[0]/OutputTracking_force[1]);
	
    integrand = 0.0;
    integrand += (SimTK::square((OutputTrackingout - COMinGround).norm()));  
}

void MocoOutputTrackingGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;
    if (get_divide_by_displacement()) {
        cost[0] /=
            calcSystemDisplacement(input.initial_state, input.final_state);
    }
}
