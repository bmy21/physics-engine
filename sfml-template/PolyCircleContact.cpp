#include "PolyCircleContact.h"

PolyCircleContact::PolyCircleContact(ConvexPolygon* p, Circle* c, const vec2& localPoint, const PhysicsSettings& ps):
	p(p), c(c), localPoint(localPoint),
	ContactConstraint(ps, p, c)
{

}

void PolyCircleContact::draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text)
{

}

bool PolyCircleContact::matches(const PolyCircleContact* other) const
{
	return (p == other->p && c == other->c);
}

void PolyCircleContact::onRebuildFrom(ContactConstraint* other)
{

}

void PolyCircleContact::initPoints()
{
	vec2 closest = transform(localPoint, p->position(), p->angle());
	vec2 disp = c->position() - closest;

	ContactPoint cp;
	cp.point = closest;
	cp.penetration = magnitude(disp) - c->radius();
}

void PolyCircleContact::rebuildPoint(ContactPoint& cp)
{

}

void PolyCircleContact::updateNormal()
{
	vec2 closest = transform(localPoint, p->position(), p->angle());
	vec2 disp = c->position() - closest;
	n = normalise(disp);
}
