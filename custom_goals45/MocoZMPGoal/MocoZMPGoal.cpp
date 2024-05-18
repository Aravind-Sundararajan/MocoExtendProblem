/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoZMPGoal.cpp                                              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Varun Joshi                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include "MocoZMPGoal.h"
#include <OpenSim/Actuators/ModelOperators.h>
using namespace OpenSim;
#define tolerance std::numeric_limits<float>::epsilon()

void MocoZMPGoal::constructProperties() {
	constructProperty_exponent(2);
}

void MocoZMPGoal::initializeOnModelImpl(const Model& model) const {
    setRequirements(1, 1);

	int exponent = get_exponent();

	// The pow() function gives slightly different results than x * x. On Mac,
    // using x * x requires fewer solver iterations.
    if (exponent == 1) {
        m_power_function = [](const double& x) { return x; };
    } else if (exponent == 2) {
        m_power_function = [](const double& x) { return x * x; };
    } else {
        m_power_function = [exponent](const double& x) {
            return pow(std::abs(x), exponent);
        };
    }
}

void MocoZMPGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
	integrand = 0.0;
	supportData out;
    getModel().realizeAcceleration(input.state);
	
	SimTK::Vec3 base_of_support(0.0); 
	auto mass_center = getModel().calcMassCenterPosition(input.state);
	for (const auto& f : getModel().getComponentList<OpenSim::Force>()) {
		if (f.getConcreteClassName() == "SmoothSphereHalfSpaceForce"){
			auto& fc = dynamic_cast<const OpenSim::SmoothSphereHalfSpaceForce&>(f);
			auto b = fc.getConnectee<ContactSphere>("sphere").getFrame().getMobilizedBodyIndex();
			OpenSim::Array<double> f_ext = f.getRecordValues(input.state);
			SimTK::Vec3 position = fc.getConnectee<OpenSim::ContactSphere>("sphere").get_location();
			SimTK::Vec3 location = fc.getConnectee<OpenSim::ContactSphere>("sphere").getFrame().getPositionInGround(input.state);
			auto position_in_ground = fc.getConnectee<OpenSim::ContactSphere>("sphere").getFrame().findStationLocationInGround(input.state, position);
			if (out.cop.find(b) == out.cop.end()) { // init struct values to 0 at mat and compute frame jacobian
				out.force[b] = SimTK::SpatialVec(2);
				out.force[b][0] = 0;
				out.force[b][1] = 0;
				out.cop[b] = SimTK::Vec3(0.0);
			}

			//Forces
			out.force[b][1][0] += f_ext[6]; 
			out.force[b][1][1] += f_ext[7];
			out.force[b][1][2] += f_ext[8];

			//Moments
			out.force[b][0][0] += out.force[b][1][1] * position_in_ground[0]; // supposed to ignore the Y since the cop is on the floor
			out.force[b][0][1] += 0;
			out.force[b][0][2] += out.force[b][1][1] * position_in_ground[2]; // supposed to ignore the Y since the cop is on the floor
		}
	}
	int counter = 0;
	for(auto it = out.cop.begin(); it != out.cop.end(); ++it) {//loop over keys i map and update center of pressure
		auto k = it->first; // keys are MobilizedBodyIndex
		if (FlattenSpatialVec(out.force[k]).norm() > tolerance){
			counter= counter + 1;
			out.cop[k][0] = out.force[k][0][0] / out.force[k][1][1]; // Mx / Fy
			out.cop[k][1] = 0.0;
			out.cop[k][2] = out.force[k][0][2] / out.force[k][1][1]; // Mz / Fy
			base_of_support = base_of_support + out.cop[k]; //find bos by averaging COPs
		} 
	}
	base_of_support = (1.0/ double(counter)) * base_of_support;
	mass_center[1] = 0.0; //zero out mass center
	base_of_support[1] = 0.0;//zero out bos
	//whole body mass center should track the base_of_support
    integrand += m_power_function((base_of_support - mass_center).norm());  
}

void MocoZMPGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;
}

SimTK::Matrix MocoZMPGoal::FlattenSpatialVec(const SimTK::SpatialVec& S) const {
    // turn a spatialvec into a 6x1 matrix.
    SimTK::Matrix spatialVecFlat(6, 1, 0.0);
    spatialVecFlat[0] = S[0][0];
    spatialVecFlat[1] = S[0][1];
    spatialVecFlat[2] = S[0][2];
    spatialVecFlat[3] = S[1][0];
    spatialVecFlat[4] = S[1][1];
    spatialVecFlat[5] = S[1][2];
    return spatialVecFlat;
}
