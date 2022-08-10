#pragma once
#include "Constraint.h"


//MouseConstraint(RigidBody* rb, const MouseHandler& mh, const PhysicsSettings& ps,
//const vec2& localPoint, real tOsc, real dampingRatio, real fMax);

class DistanceConstraint : public Constraint
{
public:
	DistanceConstraint(RigidBody* rb1, RigidBody* rb2, const vec2& localPoint1, const vec2& localPoint2, real dist,
		const PhysicsSettings& ps);

	void correctVel() override;
	void correctPos() override;
	void warmStart() override;
	void prepareVelSolver() override;

private:
	void updateCachedData();

	RigidBody* rb1;
	RigidBody* rb2;

	vec2 localPoint1, localPoint2;
	vec2 globalPoint1, globalPoint2;
	vec2 n;

	real crossFactor1 = 0, crossFactor2 = 0;
	real massFactor = 0;

	real dist = 0;

	real accLam = 0;
};