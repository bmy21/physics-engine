#pragma once
#include "Constraint.h"
class SoftDistanceConstraint : public Constraint
{
public:

	SoftDistanceConstraint(RigidBody* rb, const vec2& fixedPoint, const vec2& localPoint, real d, real tOsc, real dampingRatio, real dtInv);

	void correctVel() override;
	void correctPos() override;
	void warmStart() override;
	void updateCache() override;


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

	real crossFactor1 = 0, crossFactor2 = 0;
	real muInv1 = 0, muInv2 = 0;
	vec2 dir1, dir2;
	vec2 globalPoint;


	real beta = 0, gamma = 0;
	real C1 = 0, C2 = 0;
	real A11 = 0, A22 = 0, A12 = 0;
	real det = 0;
};

