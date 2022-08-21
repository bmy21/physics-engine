#include "LineConstraint.h"
#include "RigidBody.h"

LineConstraint::LineConstraint(RigidBody* rb1, RigidBody* rb2, const vec2& localPoint1, const vec2& localPoint2, 
	const vec2& localDir1, const PhysicsSettings& ps, bool relativeToRefPoints):
	TwoBodyConstraint(rb1, rb2, ps),
	localPoint1(localPoint1), localPoint2(localPoint2), localDir1(localDir1)
{
	localPerp1 = perp(localDir1);

	if (relativeToRefPoints)
	{
		this->localPoint1 += rb1->getRefPoint();
		this->localPoint2 += rb2->getRefPoint();
	}
}


void LineConstraint::updateCachedData()
{
	vec2 globalPoint1 = rb1->pointToGlobal(localPoint1);
	vec2 globalPoint2 = rb2->pointToGlobal(localPoint2);
	vec2 globalPerp1 = rb1->vecToGlobal(localPerp1);

	if (!isZero(globalPerp1))
	{
		globalPerp1 = normalise(globalPerp1);
	}

	real crossFactor1 = zcross(globalPerp1, rb1->position() - globalPoint2);
	real crossFactor2 = zcross(globalPerp1, rb2->position() - globalPoint2);

	C = dot(globalPoint1 - globalPoint2, globalPerp1);
	gradC = { globalPerp1.x, globalPerp1.y, crossFactor1, -globalPerp1.x, -globalPerp1.y, -crossFactor2 };

	massFactor = rb1->mInv() + rb1->IInv() * crossFactor1 * crossFactor1 + rb2->mInv() + rb2->IInv() * crossFactor2 * crossFactor2;
}
