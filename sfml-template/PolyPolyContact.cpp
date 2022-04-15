#include "PolyPolyContact.h"

PolyPolyContact::PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, int refEdgeIndex, int incEdgeIndex):
	ref(ref),
	inc(inc),
	refEdgeIndex(refEdgeIndex),
	incEdgeIndex(incEdgeIndex)
{
	vec2 refEdge = ref->transformedEdge(refEdgeIndex);
	vec2 refPoint1 = ref->transformedPoint(refEdgeIndex);
	vec2 refPoint2 = refPoint1 + refEdge;

	vec2 incEdge = inc->transformedEdge(incEdgeIndex);
	vec2 incPoint1 = inc->transformedPoint(incEdgeIndex);
	vec2 incPoint2 = incPoint1 + incEdge;

	// Clip incident edge against first side edge of reference edge
	contactPoints = clip(refEdge, refPoint2, incPoint1, incPoint2);

	// Clip again against other side edge
	if (contactPoints.size() == 2)
	{
		contactPoints = clip(-refEdge, refPoint1, contactPoints[0], contactPoints[1]);
	}
	
	vec2 normal = ref->transformedNormal(refEdgeIndex);

	for (auto it = contactPoints.begin(); it != contactPoints.end(); )
	{
		real penetration = dot(*it - refPoint1, normal);

		if (penetration > 0)
		{
			it = contactPoints.erase(it);
		}
		else
		{
			++it;
		}
	}

	// TODO: Project contact points onto the reference edge?

	//std::cout << '\n';

	// if (contactPoints.empty())	std::cout << "!!!" << '\n';
}

PolyPolyContact::~PolyPolyContact()
{
	//std::cout << "~PolyPolyContact()\n";
}

void PolyPolyContact::correctVel()
{
}

void PolyPolyContact::correctPos()
{
}

void PolyPolyContact::draw(sf::RenderWindow& window, real pixPerUnit, real fraction)
{
	vec2 refPoint1 = ref->transformedPoint(refEdgeIndex) * pixPerUnit;
	vec2 refPoint2 = refPoint1 + ref->transformedEdge(refEdgeIndex) * pixPerUnit;
	drawLine(window, refPoint1, refPoint2, sf::Color::Red);
	
	vec2 incPoint1 = inc->transformedPoint(incEdgeIndex) * pixPerUnit;
	vec2 incPoint2 = incPoint1 + inc->transformedEdge(incEdgeIndex) * pixPerUnit;
	drawLine(window, incPoint1, incPoint2, sf::Color::Green);


	real rad = 7;
	sf::CircleShape circle(rad);
	circle.setOrigin(rad, rad);
	circle.setFillColor(sf::Color::Black);
	circle.setOutlineColor(sf::Color::Black);
	circle.setOutlineThickness(-1);

	for (const vec2& cp : contactPoints)
	{
		circle.setPosition(cp*pixPerUnit);
		window.draw(circle);
	}
}
