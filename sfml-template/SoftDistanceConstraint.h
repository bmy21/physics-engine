#pragma once
#include "Constraint.h"
class SoftDistanceConstraint : public Constraint
{
public:

	SoftDistanceConstraint(RigidBody* rb, const vec2& fixedPoint, const vec2& localPoint, real d, real tOsc, real dampingRatio, real dtInv);

	void correctVel() override;
	void correctPos() override;
	void warmStart() override;


	// TODO: automatic mouse position update
	// TODO: caching of geometrical factors
	// TODO: store dt instead of dtInv!

	RigidBody* rb = nullptr;
	vec2 fixedPoint, localPoint;
	real d = 0, k = 0, b = 0, dtInv = 0;


	vec2 storedVel;
	real storedAngVel = 0;

private:
	real accLam1 = 0, accLam2 = 0;
};

