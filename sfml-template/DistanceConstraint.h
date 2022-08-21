#pragma once
#include "TwoBodyConstraint.h"

class DistanceConstraint : public TwoBodyConstraint
{
public:
	DistanceConstraint(RigidBody* rb1, RigidBody* rb2, const vec2& localPoint1, const vec2& localPoint2, real dist,
		const PhysicsSettings& ps, bool relativeToRefPoints = false);

private:
	void updateCachedData() override;

	vec2 localPoint1, localPoint2;
	vec2 globalPoint1, globalPoint2;

	real dist = 0;
};