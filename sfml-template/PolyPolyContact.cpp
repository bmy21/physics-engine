#include "PolyPolyContact.h"

PolyPolyContact::PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, const Edge* refEdge, const Edge* incEdge, const PhysicsSettings& ps):
	ref(ref), inc(inc),
	refEdge(refEdge),incEdge(incEdge),
	ContactConstraint(ps, ref, inc)
{
	localNormal = ref->vecToLocal(refEdge->normal()); 
	localRefPoint = ref->pointToLocal(refEdge->point1());
}

void PolyPolyContact::draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text)
{
	// std::cout << numPersist << '\n';

	vec2 refPoint1 = refEdge->point1() * pixPerUnit;
	vec2 refPoint2 = refEdge->point2() * pixPerUnit;

	vec2 incPoint1 = incEdge->point1() * pixPerUnit;
	vec2 incPoint2 = incEdge->point2() * pixPerUnit;

	drawThickLine(window, refPoint1, refPoint2, 3, sf::Color::Red);
	drawThickLine(window, incPoint1, incPoint2, 3, sf::Color::Green);

	real rad = 5;
	sf::CircleShape circle(rad);
	circle.setOrigin(rad, rad);
	circle.setFillColor(sf::Color::Magenta);

	for (const auto& cp : contactPoints)
	{
		circle.setPosition(cp.point.x*pixPerUnit, cp.point.y*pixPerUnit);
		window.draw(circle);

		if (debug && text)
		{
			text->setCharacterSize(40);
			text->setFillColor(sf::Color::Magenta);

			text->setString("\n\n" + cp.idAsString());

			text->setPosition(cp.point*pixPerUnit);
			centre(*text);

			window.draw(*text);
		}
	}
}

bool PolyPolyContact::matches(const PolyPolyContact* other) const
{
	if (!idsMatch(other) || ncp != other->ncp || ncp == 0)
	{
		return false;
	}

	auto& cp = contactPoints;
	auto& cpOther = other->contactPoints;
	
	// Assumes that the contact points are ordered consistently in both constraints,
	// i.e. cannot have 0 matching with 1 and 1 matching with 0
	for (int i = 0; i < ncp; ++i)
	{
		if (!cp[i].matches(cpOther[i]))
		{
			return false;
		}
	}

	return true;
}

void PolyPolyContact::updateNormal()
{
	n = ref->vecToGlobal(localNormal);
}

void PolyPolyContact::initPoints()
{
	vec2 refPoint1 = refEdge->point1();
	vec2 refPoint2 = refEdge->point2();

	vec2 incPoint1 = incEdge->point1();
	vec2 incPoint2 = incEdge->point2();

	ContactPoint cp1, cp2;
	cp1.incPointIndex = incEdge->v1index();
	cp1.refEdgeIndex = refEdge->index();
	cp1.point = incPoint1;

	cp2.incPointIndex = incEdge->v2index();
	cp2.refEdgeIndex = refEdge->index();
	cp2.point = incPoint2;

	vec2 clipNormal = normalise(refEdge->global());
	bool OK1 = clip(-clipNormal, refPoint1, ps.clipPlaneEpsilon, cp1, cp2);
	bool OK2 = clip(clipNormal, refPoint2, ps.clipPlaneEpsilon, cp1, cp2);

	updateNormal();
	checkAndAddPoint(cp1, refPoint1, ps.clipPlaneEpsilon);
	checkAndAddPoint(cp2, refPoint1, ps.clipPlaneEpsilon);
}

void PolyPolyContact::rebuildPoint(ContactPoint& cp)
{
	vec2 refPoint = ref->pointToGlobal(localRefPoint);
	cp.point = inc->pointToGlobal(cp.localIncPoint);
	cp.penetration = dot(cp.point - refPoint, n);

	// Project onto reference edge
	cp.point -= cp.penetration * n;
}

void PolyPolyContact::checkAndAddPoint(ContactPoint& cp, const vec2& ref, real eps)
{
	// If cp lies inside the reference edge, store its penetration, project it into the edge, 
	// and add it to the contactPoints vector.

	cp.penetration = dot(cp.point - ref, n);
	if (cp.penetration <= eps)
	{
		cp.localIncPoint = inc->pointToLocal(cp.point);
		cp.point -= cp.penetration * n;
		contactPoints.push_back(cp);
	}
}