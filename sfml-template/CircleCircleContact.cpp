#include "CircleCircleContact.h"

CircleCircleContact::CircleCircleContact(Circle* c1, Circle* c2, const PhysicsSettings& ps):
	c1(c1), 
	c2(c2),
	ContactConstraint(ps, c1, c2)
{
	rebuild();

	real vRel = dot(c2->pointVel(cp.point) - c1->pointVel(cp.point), n);
	cp.vRelTarget = vRel < -ps.vRelThreshold ? -e * vRel : 0;
}

void CircleCircleContact::warmStart()
{
	warmStartPoint(cp);
}

void CircleCircleContact::correctVel()
{
	solvePointFriction(cp);
	solvePointVel(cp);
}

void CircleCircleContact::correctPos()
{
	solvePointPos(cp);
}

void CircleCircleContact::draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text)
{

}

bool CircleCircleContact::matches(const CircleCircleContact* other) const
{
	// Simply match based on colliding circles, as there can only be one contact point
	return c1 == other->c1 && c2 == other->c2;
}

void CircleCircleContact::rebuild()
{
	n = normalise(c2->position() - c1->position());

	// Place the contact point in the middle of the colliding region
	vec2 furthestPoint1 = c1->position() + n * c1->radius();
	vec2 furthestPoint2 = c2->position() - n * c2->radius();
	cp.point = static_cast<real>(0.5) * (furthestPoint1 + furthestPoint2);
	cp.penetration = dot(furthestPoint2 - furthestPoint1, n);
}

void CircleCircleContact::rebuildFrom(ContactConstraint* other)
{
	// This function should only be called if *other is known to match *this
	// *other may be left in an invalid state

	CircleCircleContact* ccOther = static_cast<CircleCircleContact*>(other);
	cp.vRelTarget = ccOther->cp.vRelTarget;
}

void CircleCircleContact::updateCache()
{
	n = c2->position() - c1->position();
	
	if (magnitude(n) != 0)
	{
		n = normalise(n);
	}

	t = perp(n);

	updatePointCache(cp);
}

