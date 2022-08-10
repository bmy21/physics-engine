#pragma once
#include "Constraint.h"
class LineConstraint : public Constraint
{
public:
	LineConstraint(RigidBody* rb1, RigidBody* rb2, const vec2& localPoint1, const vec2& localPoint2, vec2 localDir1,
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

	vec2 localDir1, localPerp1;
	vec2 globalPerp1;

	real crossFactor1 = 0, crossFactor2 = 0;
	real massFactor = 0;

	real accLam = 0;
};

