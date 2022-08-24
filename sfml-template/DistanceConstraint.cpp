#include "DistanceConstraint.h"
#include "RigidBody.h"

DistanceConstraint::DistanceConstraint(RigidBody* rb1, RigidBody* rb2, const vec2& localPoint1, const vec2& localPoint2, 
	real dist, const PhysicsSettings& ps, bool relativeToRefPoints):
	TwoBodyConstraint(rb1, rb2, ps),
	localPoint1(localPoint1), localPoint2(localPoint2), dist(dist)
{
	setTarget(dist);

	if (relativeToRefPoints)
	{
		this->localPoint1 += rb1->getRefPoint();
		this->localPoint2 += rb2->getRefPoint();
	}
}

void DistanceConstraint::draw(sf::RenderWindow& window, real fraction)
{
	real halfLengthPix = 6;
	real thickness = 2;

	vec2 p1 = rb1->pointToGlobal(localPoint1) * ps.pixPerUnit;
	vec2 p2 = rb2->pointToGlobal(localPoint2) * ps.pixPerUnit;

	drawThickLine(window, p1, p2, thickness, sf::Color::Red);
}

void DistanceConstraint::updateCachedData()
{
	globalPoint1 = rb1->pointToGlobal(localPoint1);
	globalPoint2 = rb2->pointToGlobal(localPoint2);

	vec2 n = globalPoint1 - globalPoint2;

	C = magnitude(n);

	if (!isZero(n))
	{
		n = normalise(n);
	}

	real crossFactor1 = zcross(globalPoint1 - rb1->position(), n);
	real crossFactor2 = zcross(globalPoint2 - rb2->position(), n);

	gradC = { n.x, n.y, crossFactor1, -n.x, -n.y, -crossFactor2 };
	
	massFactor = rb1->mInv() + rb1->IInv() * crossFactor1 * crossFactor1 + rb2->mInv() + rb2->IInv() * crossFactor2 * crossFactor2;
}
