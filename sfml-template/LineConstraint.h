#pragma once
#include "TwoBodyConstraint.h"

class LineConstraint : public TwoBodyConstraint
{
public:
	LineConstraint(RigidBody* rb1, RigidBody* rb2, const vec2& localPoint1, const vec2& localPoint2, const vec2& localDir1,
		const PhysicsSettings& ps);

private:
	void updateCachedData();

	vec2 localPoint1, localPoint2;
	vec2 localDir1, localPerp1;
};

