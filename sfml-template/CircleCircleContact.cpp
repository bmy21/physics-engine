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
	// Simply match based on colliding circles, as there can only be one contact point
	return c1 == other->c1 && c2 == other->c2;
}

void CircleCircleContact::initPoints()
{
	contactPoints.resize(1);
	rebuildPoint(contactPoints.front());
}

void CircleCircleContact::rebuildPoint(ContactPoint& cp)
{
	updateNormal(); // unnecessary?

	// Place the contact point in the middle of the colliding region
	vec2 furthestPoint1 = c1->furthestPoint(n);
	vec2 furthestPoint2 = c2->furthestPoint(-n);

	cp.point = static_cast<real>(0.5) * (furthestPoint1 + furthestPoint2);
	cp.penetration = dot(furthestPoint2 - furthestPoint1, n);
}

void CircleCircleContact::onRebuildFrom(ContactConstraint* other)
{
	
}

void CircleCircleContact::updateNormal()
{
	n = c2->position() - c1->position();

	if (magnitude(n) != 0)
	{
		n = normalise(n);
	}
}