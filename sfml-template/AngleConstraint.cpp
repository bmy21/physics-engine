#include "AngleConstraint.h"
#include "RigidBody.h"

AngleConstraint::AngleConstraint(RigidBody* rb1, RigidBody* rb2, real angleDiff, const PhysicsSettings& ps):
	TwoBodyConstraint(rb1, rb2, ps),
	angleDiff(angleDiff)
{
	gradC = { 0, 0, 1, 0, 0, -1 };
	massFactor = rb1->IInv() + rb2->IInv();
}

void AngleConstraint::updateCachedData()
{
	// gradC and massFactor are constant
	C = rb1->angle() - rb2->angle();
}
