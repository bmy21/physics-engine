#include "PolyPolyContact.h"

PolyPolyContact::PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, int refEdgeIndex, int incEdgeIndex, int incPointIndex):
	ref(ref),
	inc(inc),
	refEdgeIndex(refEdgeIndex),
	incEdgeIndex(incEdgeIndex),
	incPointIndex(incPointIndex)
{
	vec2 refEdge = ref->transformedEdge(refEdgeIndex);
	vec2 refPoint1 = ref->transformedPoint(refEdgeIndex);
	vec2 refPoint2 = refPoint1 + refEdge;

	vec2 incEdge = inc->transformedEdge(incEdgeIndex);
	vec2 incPoint1 = inc->transformedPoint(incEdgeIndex);
	vec2 incPoint2 = incPoint1 + incEdge;

	// Clip incident edge against first side of reference edge
	int nclips1 = -1, nclips2 = -1;
	auto cpCoords = clip(-refEdge, refPoint1, incPoint1, incPoint2, nclips1);
	assert(nclips1 == 0 || nclips1 == 1);

	// Clip again against other side edge
	if (true) // nclips1 == 0)
	{
		cpCoords = clip(refEdge, refPoint2, cpCoords[0], cpCoords[1], nclips2);
		assert(nclips2 == 0 || nclips2 == 1);
	}
	
	vec2 normal = ref->transformedNormal(refEdgeIndex);

	for (auto it = cpCoords.begin(); it != cpCoords.end(); ++it)
	{
		real penetration = dot(*it - refPoint1, normal);

		if (penetration <= 0)
		{
			ContactPoint cp;
			cp.penetration = penetration;
			cp.point = *it;
			contactPoints.push_back(cp);
		}
	}

	ncp = contactPoints.size();
	assert(ncp == 1 || ncp == 2);

	/*if (ncp == 1)
	{
		auto& cp = contactPoints[0];

		cp.typeA = FeatureType::Edge;
		cp.indexA = refEdgeIndex;

		cp.typeB = FeatureType::Point;
		cp.indexB = incPointIndex;
	}
	else
	{
		if (nclips1 == 0 && nclips2 == 0)
		{

		}
	}*/



	// TODO: Project contact points onto the reference edge?
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

	for (const auto& cp : contactPoints)
	{
		circle.setPosition(cp.point.x*pixPerUnit, cp.point.y*pixPerUnit);
		window.draw(circle);
	}
}
