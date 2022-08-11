#pragma once
#include "TwoBodyConstraint.h"

class AngleConstraint : public TwoBodyConstraint
{
public:
	AngleConstraint(RigidBody* rb1, RigidBody* rb2, real angleDiff, const PhysicsSettings& ps);

private:
	void updateCachedData() override;
	real angleDiff = 0;
};
