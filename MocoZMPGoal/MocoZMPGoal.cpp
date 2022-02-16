/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoZMPGoal.cpp                                              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoZMPGoal.h"

using namespace OpenSim;

void MocoZMPGoal::constructProperties() {
    constructProperty_divide_by_displacement(false);
}

void MocoZMPGoal::initializeOnModelImpl(const Model& model) const {
    setRequirements(1, 1);
}

void MocoZMPGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    getModel().realizeDynamics(input.state);
	auto m_model = getModel();
 	SimTK::Vec3 PelvisInGround(0.0, 0.0, 0.0);
    SimTK::Vec3 COMinGround(0.0, 0.0, 0.0);
	
	COMinGround =m_model.getMultibodySystem().getMatterSubsystem().calcSystemMassCenterLocationInGround(input.state);
	COMinGround[1] = 0.0;
	// Calculate ZMP
		
	/* Solve the inverse dynamics system of equations for generalized coordinate forces, Tau.
	Applied loads are explicity provided as generalized coordinate forces (MobilityForces)
	and/or a Vector of Spatial-body forces */
	const ForceSet& forces = getModel().getForceSet();
	int nf = forces.getSize();
    auto myState(input.state);
	for(int f=0;f<nf;f++) {
	getModel().getForceSet()[f].setAppliesForce(myState,true); // Turning off all other forces except gravity
	}
    getModel().realizeDynamics(myState);
	SimTK::Vector_<SimTK::SpatialVec> appliedBodyForces = getModel().getMultibodySystem().getRigidBodyForces(input.state, SimTK::Stage::Dynamics);
	// Removing body forces such as contacts
	
	SimTK::Vector appliedMobilityForces(m_model.updMatterSubsystem().getNumMobilities());
	// Removing mobility forces
	appliedMobilityForces *=0;
	
	// Results of the inverse dynamics for the generalized forces to satisfy accelerations
	SimTK::Vector residualMobilityForces;
	SimTK::Vector knownUdot = input.state.getUDot();
 
	// Perform inverse dynamics
	getModel().getMultibodySystem().getMatterSubsystem().calcResidualForceIgnoringConstraints(input.state, appliedMobilityForces, appliedBodyForces, knownUdot, residualMobilityForces);
	
	/* Get model free floating joint (Pelvis Joint or the first joint conncted to the ground)*/
	SimTK::SpatialVec equivalentBodyForcesAtPelvis = getModel().getJointSet()[0].calcEquivalentSpatialForce(input.state, residualMobilityForces);
	
	for(int f=0;f<nf;f++) {
	getModel().getForceSet()[f].setAppliesForce(myState,false); // Turning on all other forces again
	}
	getModel().realizeDynamics(myState);
	
	SimTK::Vec3 pelvis_moment(0.0, 0.0, 0.0);
	SimTK::Vec3 pelvis_force(0.0, 0.0, 0.0);
	
	for(int n=0;n<3;n++){ 
	pelvis_moment[n] = equivalentBodyForcesAtPelvis[0][n];
	pelvis_force[n] = equivalentBodyForcesAtPelvis[1][n];
	}
	
	auto PelvisBody  = m_model.getBodySet().get("pelvis");
	SimTK::Vec3 PelvisCoM;
	PelvisCoM = PelvisBody.getMassCenter();
	const  SimbodyEngine& engine = getModel().getSimbodyEngine();
	engine.transformPosition(input.state, PelvisBody, PelvisCoM, PelvisInGround);
	
	SimTK::Vec3 zmp_moment(0.0, 0.0, 0.0);
	zmp_moment = pelvis_moment + PelvisInGround % pelvis_force;
	SimTK::Vec3 zmp_force(0.0, 0.0, 0.0);
	zmp_force[0] = pelvis_force[0];
	zmp_force[1] = pelvis_force[1];
	zmp_force[2] = pelvis_force[2];
	
    SimTK::Vec3 zmpout(0.0, 0.0, 0.0);
	zmpout[0] = zmp_moment[2]/zmp_force[1];
	zmpout[1] = 0.0;
	zmpout[2] = -1.0 * (zmp_moment[0]/zmp_force[1]);
	
    integrand = 0.0;
    integrand += (SimTK::square((zmpout - COMinGround).norm()));  
}

void MocoZMPGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;
    if (get_divide_by_displacement()) {
        cost[0] /=
            calcSystemDisplacement(input.initial_state, input.final_state);
    }
}
