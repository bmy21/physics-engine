#pragma once
#include "Constraint.h"
#include "MouseHandler.h"

class MouseConstraint : public Constraint
{
public:
	MouseConstraint(RigidBody* rb, const MouseHandler* mh, const PhysicsSettings& ps,
		const vec2& localPoint, real tOsc, real dampingRatio, real fMax);

	void correctVel() override;
	void correctPos() override;
	void warmStart() override;
	void updateCache() override;

	void calculateParams();

	RigidBody* rb = nullptr;
	vec2 localPoint;


private:
	const MouseHandler* mh;

	real k = 0, b = 0, fMax = 0;
	real beta = 0, gamma = 0;

	real C1 = 0, C2 = 0;
	real A11 = 0, A22 = 0, A12 = 0;
	real det = 0;

	real crossFactor1 = 0, crossFactor2 = 0;
	real muInv1 = 0, muInv2 = 0;
	vec2 dir1, dir2;
	vec2 globalPoint;

	real accLam1 = 0, accLam2 = 0;
};

