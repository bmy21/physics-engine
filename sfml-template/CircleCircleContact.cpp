#include "CircleCircleContact.h"

CircleCircleContact::CircleCircleContact(Circle* c1, Circle* c2, const PhysicsSettings& ps):
	c1(c1), c2(c2),
	ContactConstraint(ps, c1, c2)
{
	
}


void CircleCircleContact::draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text)
{

}

bool CircleCircleContact::matches(const CircleCircleContact* other) const
{
	// Match based on which circles are colliding, as there can only be one contact point
	return idsMatch(other);
}

void CircleCircleContact::initPoints()
{
	updateNormal();
	contactPoints.resize(1);
	rebuildPoint(contactPoints.front());
}

void CircleCircleContact::rebuildPoint(ContactPoint& cp)
{
	// Place the contact point in the middle of the colliding region
	vec2 furthestPoint1 = c1->furthestPoint(n);
	vec2 furthestPoint2 = c2->furthestPoint(-n);

	cp.point = static_cast<real>(0.5) * (furthestPoint1 + furthestPoint2);
	cp.penetration = dot(furthestPoint2 - furthestPoint1, n);
}

void CircleCircleContact::updateNormal()
{
	n = c2->position() - c1->position();
	
	if (!isZero(n))
	{
		n = normalise(n);
	}
}