#include "PolyCircleContact.h"

PolyCircleContact::PolyCircleContact(ConvexPolygon* p, Circle* c, 
	const vec2& localNormal, const vec2& localRefPoint, Voronoi region, const PhysicsSettings& ps):
	p(p), c(c), localNormal(localNormal), localRefPoint(localRefPoint), region(region),
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
	return (p->id == other->p->id && c->id == other->c->id);
}

void PolyCircleContact::onRebuildFrom(ContactConstraint* other)
{
	PolyCircleContact* pcOther = static_cast<PolyCircleContact*>(other);

	localNormal = pcOther->localNormal;
	localRefPoint = pcOther->localRefPoint;
	region = pcOther->region;
}

void PolyCircleContact::initPoints()
{
	updateNormal();
	contactPoints.resize(1);
	rebuildPoint(contactPoints.front());
}

void PolyCircleContact::rebuildPoint(ContactPoint& cp)
{
	vec2 globalRefPoint = p->pointToGlobal(localRefPoint);
	vec2 circlePoint = c->furthestPoint(-n);

	cp.penetration = dot(circlePoint - globalRefPoint, n);
	cp.point = circlePoint - cp.penetration * n;
}

void PolyCircleContact::updateNormal()
{
	if (region == Voronoi::Edge || region == Voronoi::Inside)
	{
		n = p->vecToGlobal(localNormal);
	}
	else
	{
		// Vertex
		n = c->position() - p->pointToGlobal(localRefPoint);
		if (magSquared(n) != 0)
		{
			n = normalise(n);
		}
	}
}
