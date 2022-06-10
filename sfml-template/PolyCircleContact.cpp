#include "PolyCircleContact.h"

PolyCircleContact::PolyCircleContact(ConvexPolygon* p, Circle* c, 
	const vec2& localNormal, const vec2& localRefPoint, const PhysicsSettings& ps):
	p(p), c(c), localNormal(localNormal), localRefPoint(localRefPoint),
	ContactConstraint(ps, p, c)
{

}

void PolyCircleContact::draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text)
{
	real rad = 5;
	sf::CircleShape circle(rad);
	circle.setOrigin(rad, rad);
	circle.setFillColor(sf::Color::Magenta);

	for (const auto& cp : contactPoints)
	{
		circle.setPosition(cp.point.x * pixPerUnit, cp.point.y * pixPerUnit);
		window.draw(circle);

		if (debug && text)
		{
			text->setCharacterSize(40);
			text->setFillColor(sf::Color::Magenta);

			text->setString("\n\n" + cp.idAsString());

			text->setPosition(cp.point * pixPerUnit);
			centre(*text);

			window.draw(*text);
		}
	}
}

bool PolyCircleContact::matches(const PolyCircleContact* other) const
{
	return (p == other->p && c == other->c);
}

void PolyCircleContact::onRebuildFrom(ContactConstraint* other)
{
	PolyCircleContact* pcOther = static_cast<PolyCircleContact*>(other);

	localNormal = pcOther->localNormal;
	localRefPoint = pcOther->localRefPoint;
}

void PolyCircleContact::initPoints()
{
	// TODO: should depend on edge or vertex contact
	updateNormal();
	contactPoints.resize(1);
	rebuildPoint(contactPoints.front());
}

void PolyCircleContact::rebuildPoint(ContactPoint& cp)
{
	// TODO: does this calculation work for vertex contact?

	vec2 globalRefPoint = p->pointToGlobal(localRefPoint);
	vec2 circlePoint = c->furthestPoint(-n);

	cp.penetration = dot(circlePoint - globalRefPoint, n);
	cp.point = circlePoint - cp.penetration * n;
}

void PolyCircleContact::updateNormal()
{
	// TODO: should depend on edge or vertex contact
	n = p->vecToGlobal(localNormal);

	//vec2 closest = transform(localPoint, p->position(), p->angle());
	//vec2 disp = c->position() - closest;
	//n = normalise(disp);
}
