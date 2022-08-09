#pragma once
#include "Constraint.h"


//MouseConstraint(RigidBody* rb, const MouseHandler& mh, const PhysicsSettings& ps,
//const vec2& localPoint, real tOsc, real dampingRatio, real fMax);

class DistanceConstraint : public Constraint
{
public:
	DistanceConstraint(RigidBody* rb1, RigidBody* rb2, const PhysicsSettings& ps, std::initializer_list<RigidBody*> rigidBodies);

	void correctVel() override;
	void correctPos() override;
	void warmStart() override;
	void prepareVelSolver() override;
};

//class DistanceConstraint : public Constraint
//{
//public:
//	DistanceConstraint();
//
//	void correctVel() override;
//	void correctPos() override;
//	void warmStart() override;
//
//	vec2 point;
//	RigidBody* rb;
//	
//
//private:
//	
//	real storedLambda = 0;
//
//};
//
