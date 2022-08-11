#include "DistanceConstraint.h"
#include "RigidBody.h"

DistanceConstraint::DistanceConstraint(RigidBody* rb1, RigidBody* rb2, const vec2& localPoint1, const vec2& localPoint2, 
	real dist, const PhysicsSettings& ps):
	TwoBodyConstraint(rb1, rb2, ps),
	localPoint1(localPoint1), localPoint2(localPoint2), dist(dist)
{
	
}

void DistanceConstraint::updateCachedData()
{
	globalPoint1 = rb1->pointToGlobal(localPoint1);
	globalPoint2 = rb2->pointToGlobal(localPoint2);

	vec2 n = globalPoint1 - globalPoint2;

	C = magnitude(n) - dist;

	if (!isZero(n))
	{
		n = normalise(n);
	}

	real crossFactor1 = zcross(globalPoint1 - rb1->position(), n);
	real crossFactor2 = zcross(globalPoint2 - rb2->position(), n);

	gradC = { n.x, n.y, crossFactor1, -n.x, -n.y, -crossFactor2 };
	
	massFactor = rb1->mInv() + rb1->IInv() * crossFactor1 * crossFactor1 + rb2->mInv() + rb2->IInv() * crossFactor2 * crossFactor2;
}
