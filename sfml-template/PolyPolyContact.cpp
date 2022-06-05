#include "PolyPolyContact.h"

PolyPolyContact::PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, const Edge* refEdge, const Edge* incEdge, const PhysicsSettings& ps):
	ref(ref), inc(inc),
	refEdge(refEdge),incEdge(incEdge),
	ContactConstraint(ps, ref, inc)
{
	
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
	if (ref != other->ref || inc != other->inc || ncp != other->ncp || ncp == 0)
	{
		return false;
	}

	auto& cp = contactPoints;
	auto& cpOther = other->contactPoints;
	
	// Assumes that the contact points are ordered consistently in both constraints,
	// i.e. cannot have 0 matching with 1 and 1 matching with 0
	if (ncp == 1)
	{
		return cp[0].matches(cpOther[0]);
	}
	else if (ncp == 2)
	{
		return cp[0].matches(cpOther[0]) && cp[1].matches(cpOther[1]);
	}
}

void PolyPolyContact::updateNormal()
{
	n = refEdge->normal();
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
	bool OK1 = clip(-clipNormal, refPoint1, ps.clipPlaneEpsilon, refEdge->v1index(), cp1, cp2);
	bool OK2 = clip(clipNormal, refPoint2, ps.clipPlaneEpsilon, refEdge->v2index(), cp1, cp2);

	updateNormal();
	checkAndAddPoint(cp1, refPoint1, ps.clipPlaneEpsilon);
	checkAndAddPoint(cp2, refPoint1, ps.clipPlaneEpsilon);
}

void PolyPolyContact::rebuildPoints()
{
	updateNormal();
	vec2 refPoint = refEdge->point1();

	for (auto& cp : contactPoints)
	{	
		if (cp.clippedAgainstPoint == -1)
		{
			// Wasn't clipped
			cp.point = inc->vertex(cp.incPointIndex);
		}
		else
		{
			// Find the point where the incident edge crosses the clip plane
			vec2 p = incEdge->global();
			vec2 q = n;

			vec2 a = inc->vertex(cp.incPointIndex);
			vec2 b = ref->vertex(cp.clippedAgainstPoint);

			// TODO: Quicker to do this with dot products?
			assert(zcross(q, p) != 0);
			vec2 intersect = a + p * zcross(q, b - a) / zcross(q, p);
			cp.point = intersect;
		}

		cp.penetration = dot(cp.point - refPoint, n);

		// Project onto reference edge
		cp.point -= cp.penetration * n;
	}
}

void PolyPolyContact::rebuildFrom(ContactConstraint* other)
{
	// This function should only be called if *other is known to match *this
	// *other may be left in an invalid state

	PolyPolyContact* ppOther = static_cast<PolyPolyContact*>(other);

	for (int i = 0; i < ncp; ++i)
	{
		// Updated vRelTarget is already stored in ppOther->contactPoints
		ppOther->contactPoints[i].lambda = contactPoints[i].lambda;
		ppOther->contactPoints[i].fLambda = contactPoints[i].fLambda;
	}

	contactPoints = std::move(ppOther->contactPoints);

	// TODO: "onRebuildFrom" function
	refEdge = ppOther->refEdge;
	incEdge = ppOther->incEdge;
}

void PolyPolyContact::checkAndAddPoint(ContactPoint& cp, const vec2& ref, real eps)
{
	// If cp lies inside the reference edge, store its penetration, project it into the edge, 
	// and add it to the contactPoints vector.

	real p = dot(cp.point - ref, n);

	if (p <= eps)
	{
		cp.penetration = p;
		cp.point -= cp.penetration * n;
		contactPoints.push_back(cp);
	}
}